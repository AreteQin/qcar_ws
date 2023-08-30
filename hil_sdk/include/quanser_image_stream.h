#if !defined(_quanser_image_stream_h)
#define _quanser_image_stream_h

#include "quanser_extern.h"
#include "quanser_types.h"
#include "quanser_errors.h"
#include "quanser_image.h"

/*
** Image Stream Model
**
** Each source block:
**  1. Gets an image frame from the global image pool.
**  2. Fills the image frame with data e.g. from a camera.
**  3. Pushes the image frame onto the t_image_stream stack.
**
** Each processing block:
**  1. Gets an output image frame from the global image pool.
**  2. Pops the input image frame off the t_image_stream stack. If the stack is empty then the signal has been forked. Need an Image Fork block.
**  3. Processes the input image, writing to the output image frame acquired in step 1.
**  4. Puts the input image frame back in the global image pool.
**  5. Pushes the processed output image frame onto the t_image_stream stack.
**
** Each sink block:
**  1. Pops the input image frame off the t_image_stream stack.
**  2. Processes the input image.
**  3. Puts the input image frame back in the global image pool.
**
** An Image Fork block:
**  1. Gets N-1 output image frames from the global image pool.
**  2. Pops the input image frame off the t_image_stream stack.
**  3. Copies the input image to the N-1 output images.
**  5. Pushes the input image frame to the t_image_stream stack for the first output, and pushes the N-1 output images to the t_image_stream stacks
**     of the rest of the outputs.
**
** In this scheme, a long chain of image processing blocks may only allocate two images, provided the image format and dimensions stay the same. Note that
** processing blocks should simply do steps 2 and 4 if their output is not connected. Also, the global image pool can free any images returned to the pool
** at any time, if it needs memory to allocate a new image. Also, image frames can be marked as "owned" so that the global image pool ignores those images.
** This can be used by sources that keep a single image and do not get an image from the global image pool. However, owned image frames can never be freed,
** although they can be used in the chain.
**
** In this scenario, an EGLStream is only used by individual blocks to interface between different hardware resources. Also note that a t_image_stream stack
** is not technically necessary in the scenarios above. Simple input and output fields would be enough with reference counts. However, a stack would allow
** frame rate conversions if desired, in which faster frames were even merged to produce slower frames, if desired.
**
** The problem with this approach is that an Image Fork block is required to use a t_image_stream as an input to multiple blocks. This is not
** a huge problem though as re-use of a signal can be detected and an error issued indicating the need to fork the image stream signal.
**
** The advantage is it minimizes the memory requirements quite well, only allocating image frames when absolutely necessary. For the QCar, the image frames
** can be represented as cudaEglFrame objects, likely containing a YUV420 planar image (Y plane with U plane and V plane that are half the width and height).
** However, since a cudaEglFrame describes its colour format and planes fully, multiple image formats could be supported (but are not ideal!).
*/

/*
** XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX TOO RESOURCE INTENSIVE!!! XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
** Note that a scheme that was considered, but determined to take too many resources is the following:
**
**  Each source block:
**      1. Calls get_image to get image frame from EGLStream. If no image frame is available then create a new image frame.
**      2. Fill the image frame with data e.g. from a camera.
**      3. Calls post_image to present the image to its output t_image_stream.
**
**  Each processing block:
**      1. Calls get_image on its output t_image_stream to get an image frame from the output EGLStream. If no image frame is available then create a new image frame.
**      2. Calls acquire_frame on its input t_image_stream to acquire the image frame from its input EGLStream.
**      3. Processes the input image frame, writing the results to the output image frame.
**      4. Calls release_frame on its input t_image_stream to release the image frame to its input EGLStream.
**      5. Calls post_image on its output t_image_stream to post the output image frame to the output EGLStream.
**
**      A special case occurs when the processing block's output is not connected. In that case it should simply acquire and release the
**      input image frame without doing any processing so the block providing the input frame can operate normally.
**
**  Each sink block:
**      1. Calls acquire_frame on its input t_image_stream to acquire the image frame from its input EGLStream.
**      2. Processes the input image frame.
**      3. Calls release_frame on its input t_image_stream to release the image frame to its input EGLStream.
**
**  Note that with this model, if a signal is split (one output going to multiple inputs of subsequent blocks), everything continues
**  to work as expected using the EGLStream mailbox mode because each processing block acquires and releases the image frame within
**  its execution, so after the first downstream block processes the image, the other blocks will acquire the same image, which is
**  exactly what we want.
**
**  Where the model breaks down is if the blocks are running at different rates. This model assumes all blocks are in the same
**  task (running at the same rate). However, since Simulink can confirm whether the input sample time matches the block sample
**  time, this should not be an issue in Simulink. Also, the Rate Transition block hopefully won't work with custom signal types
**  so it won't be possible to cross sample time domains in Simulink with image streams.
**
**  The problem with this approach is that every block must allocate an image frame so we will rapidly run out of memory!!!
** =====================================================================================================
*/

/*
** An image stream is an opaque image format optimized for each platform. It takes full advantage of the platform capabilities
** available, such as the GPU. For example, on an NVIDIA target, it will keep the image in the GPU without ever bringing it
** into main memory, avoiding the overhead of transferring the image to or from the GPU to CPU memory.
*/
typedef struct tag_image_stream* t_image_stream;
typedef struct tag_image_pool* t_image_pool;
typedef struct tag_image* t_image;

/*--- Image Stream functions ---*/

/*
** Description:
**
**  Creates an image stream.
**
** Parameters:
**
**  stream - the address of a t_image_stream variable to receive the handle to the image stream.
**
** Returns:
**
**  Returns zero on success and a negative error code otherwise.
*/
EXTERN t_error
image_stream_create(t_image_stream* stream);

/*
** Description:
**
**  Pushes an image onto the image stream's stack.
**
** Parameters:
**
**  stream - a t_image_stream variable containing the handle to the image stream.
**  image  - a t_image representing the image.
**
** Returns:
**
**  Returns zero on success and a negative error code otherwise.
*/
EXTERN t_error
image_stream_push_image(t_image_stream stream, t_image image);

/*
** Description:
**
**  Pops an image from the image stream's stack.
**
** Parameters:
**
**  stream - a t_image_stream variable containing the handle to the image stream.
**  image  - a pointer to an t_image variable which will receive the image. It may be NULL,
**           in which case the image is popped but discarded.
**
** Returns:
**
**  Returns zero on success and a negative error code otherwise. If the stack is empty so
**  that no image is available then -QERR_OBJECT_NOT_FOUND is returned.
*/
EXTERN t_error
image_stream_pop_image(t_image_stream stream, t_image* image);

/*
** Description:
**
**  Destroys an image stream.
**
** Parameters:
**
**  stream - a t_image_stream variable containing the handle to the image stream.
**
** Returns:
**
**  Returns zero on success and a negative error code otherwise.
*/
EXTERN t_error
image_stream_destroy(t_image_stream stream);

/*--- Image Pool functions ---*/

/*
** Description:
**
**  Creates an image pool. This function is typically called only once to get the global
**  image pool.
**
** Parameters:
**
**  pool - the address of a t_image_pool variable to receive the handle to the image pool.
**
** Returns:
**
**  Returns zero on success and a negative error code otherwise.
*/
EXTERN t_error
image_pool_create(t_image_pool* pool);

/*
** Description:
**
**  Returns the default image pool. Passing NULL as the pool to the other image_pool functions
**  will use this default pool so this function is not required.
**
** Parameters:
**
**  pool - the address of a t_image_pool variable to receive the handle to the default image pool.
**
** Returns:
**
**  Returns zero on success and a negative error code otherwise.
*/
EXTERN t_error
image_pool_get_default(t_image_pool* pool);

/*
** Description:
**
**  Gets an image from the given image pool. If pool is NULL then a default image pool
**  is used. The memory for the image will be allocated if an image of the appropriate
**  format and dimensions is not already available in the pool. Note that the image
**  belongs to the pool. The pool lifetime must exceed that of all images it creates
**  even if the images are not currently part of the pool.
**
** Parameters:
**
**  pool   - a t_image_pool variable containing the handle to the image pool. If this
**           argument is NULL then a default image pool is used.
**  format - the colour format of the image: either monochrome or colour. Further
**           flexibility is not supported. The image format used is platform-specific
**           and optimized for the particular platform.
**  width  - the width of the image to retrieve.
**  height - the height of the image to retrieve.
**  image  - a pointer to a t_image variable to receive the handle to the image.
**
** Returns:
**
**  Returns zero on success and a negative error code otherwise.
*/
EXTERN t_error
image_pool_get_image(t_image_pool pool, t_image_color_format format, t_uint width, t_uint height, t_image* image);

/*
** Description:
**
**  Returns an image to the given image pool. If pool is NULL then a default image pool
**  is used. The image should no longer be used.
**
** Parameters:
**
**  pool  - a t_image_pool variable containing the handle to the image pool. If this
**          argument is NULL then a default image pool is used.
**  image - the image to return to the pool.
**
** Returns:
**
**  Returns zero on success and a negative error code otherwise.
*/
EXTERN t_error
image_pool_put_image(t_image_pool pool, t_image image);

/*
** Description:
**
**  Frees all the image in the given image pool, releasing their resources.
**  However, it does not free the image pool itself which may still be used.
**
** Parameters:
**
**  pool - a t_image_pool variable containing the handle to the image pool. If this
**         argument is NULL then the default image pool is cleared.
**
** Returns:
**
**  Returns zero on success and a negative error code otherwise.
*/
EXTERN t_error
image_pool_clear(t_image_pool pool);

/*
** Description:
**
**  Destroys the given image pool, releasing its resources.
**
** Parameters:
**
**  pool - a t_image_pool variable containing the handle to the image pool. The argument
**         may not be NULL.
**
** Returns:
**
**  Returns zero on success and a negative error code otherwise.
*/
EXTERN t_error
image_pool_destroy(t_image_pool pool);

#endif

