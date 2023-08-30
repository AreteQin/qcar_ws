#if !defined(_quanser_video3d_h)
#define _quanser_video3d_h

#include "quanser_extern.h"
#include "quanser_video.h"

typedef struct tag_video3d * t_video3d;
typedef struct tag_video3d_stream * t_video3d_stream;
typedef struct tag_video3d_frame * t_video3d_frame;

typedef enum tag_video3d_property
{
    VIDEO3D_PROPERTY_BACKLIGHT_COMPENSATION,
    VIDEO3D_PROPERTY_BRIGHTNESS,
    VIDEO3D_PROPERTY_CONTRAST,
    VIDEO3D_PROPERTY_EXPOSURE,
    VIDEO3D_PROPERTY_GAIN,
    VIDEO3D_PROPERTY_GAMMA,
    VIDEO3D_PROPERTY_HUE,
    VIDEO3D_PROPERTY_SATURATION,
    VIDEO3D_PROPERTY_SHARPNESS,
    VIDEO3D_PROPERTY_WHITE_BALANCE,
    VIDEO3D_PROPERTY_ENABLE_AUTO_EXPOSURE,
    VIDEO3D_PROPERTY_ENABLE_AUTO_WHITE_BALANCE,
    VIDEO3D_PROPERTY_ENABLE_EMITTER,                /* enable laser emitter for IR cameras (depth feature) */
    VIDEO3D_PROPERTY_VISUAL_PRESET,                 /* presets used to control camera settings (particularly depth). See t_video3d_visual_preset below. */

    /* TODO: There are many depth camera options, but they seem to be specific to the R200 camera */

    NUMBER_OF_VIDEO3D_PROPERTIES
} t_video3d_property;

typedef enum tag_video3d_stream_type
{
    VIDEO3D_STREAM_DEPTH,
    VIDEO3D_STREAM_COLOR,
    VIDEO3D_STREAM_INFRARED,
    VIDEO3D_STREAM_FISHEYE,
    VIDEO3D_STREAM_GYROSCOPE,
    VIDEO3D_STREAM_ACCELEROMETER,
    VIDEO3D_STREAM_POSE,
    /*VIDEO3D_STREAM_GPIO,*/

    NUMBER_OF_VIDEO3D_STREAMS
} t_video3d_stream_type;

typedef enum tag_video3d_visual_preset
{
    VIDEO3D_PRESET_DEFAULT,                         /* use default camera settings */

    /* R200 presets */
    VIDEO3D_PRESET_R200_NO_OUTLIER_REMOVAL,         /* Disable almost all hardware-based outlier removal */
    VIDEO3D_PRESET_R200_LOW_OUTLIER_REMOVAL,        /* Provide a depthmap with a lower number of outliers removed, which has minimal false negatives. */
    VIDEO3D_PRESET_R200_MEDIUM_OUTLIER_REMOVAL,     /* Provide a depthmap with a medium number of outliers removed, which has balanced approach. */
    VIDEO3D_PRESET_R200_OPTIMIZED_OUTLIER_REMOVAL,  /* Provide a depthmap with a medium/high number of outliers removed. Derived from an optimization function. */
    VIDEO3D_PRESET_R200_HIGH_OUTLIER_REMOVAL,       /* Provide a depthmap with a higher number of outliers removed, which has minimal false positives. */

    /* SR300 (IVCAM) presets */
    VIDEO3D_PRESET_SR300_SHORT_RANGE,
    VIDEO3D_PRESET_SR300_MEDIUM_RANGE,
    VIDEO3D_PRESET_SR300_LONG_RANGE,
    VIDEO3D_PRESET_SR300_BACKGROUND_SEGMENTATION,
    VIDEO3D_PRESET_SR300_GESTURE_RECOGNITION,
    VIDEO3D_PRESET_SR300_OBJECT_SCANNING,
    VIDEO3D_PRESET_SR300_FACE_ANALYTICS,
    VIDEO3D_PRESET_SR300_FACE_LOGIN,
    VIDEO3D_PRESET_SR300_GR_CURSOR,
    VIDEO3D_PRESET_SR300_INFRARED_ONLY,

    /* RS400 presets */
    VIDEO3D_PRESET_RS400_HAND,
    VIDEO3D_PRESET_RS400_MEDIUM_DENSITY,
    VIDEO3D_PRESET_RS400_HIGH_DENSITY,
    VIDEO3D_PRESET_RS400_HIGH_ACCURACY,

    NUMBER_OF_VISUAL_PRESETS

} t_video3d_visual_preset;

typedef enum tag_video3d_motion_format
{
    VIDEO3D_MOTION_FORMAT_3DOF,         /* three 32-bit floating-point values in SI units e.g. m/s^2 or rad/s */
    VIDEO3D_MOTION_FORMAT_6DOF_POSE,    /* six 32-bit floating-point values in SI units representing position and orientation */

    NUMBER_OF_MOTION_FORMATS
} t_video3d_motion_format;

typedef enum tag_video3d_distortion_model
{
    VIDEO3D_DISTORTION_NONE,
    VIDEO3D_DISTORTION_MODIFIED_BROWN_CONRADY,
    VIDEO3D_DISTORTION_INVERSE_BROWN_CONRADY,
    VIDEO3D_DISTORTION_FTHETA,
    VIDEO3D_DISTORTION_BROWN_CONRADY,
    VIDEO3D_DISTORTION_KANNALA_BRANDT4,

    NUMBER_OF_VIDEO3D_DISTORTIONS
} t_video3d_distortion_model;

#if defined(_WIN32) || defined(_INTEL_AERO) || defined(_DUOVERO) || defined(_UBUNTU) || defined(_RASPBERRY_PI_3) || defined(_RASPBERRY_PI_4) || defined(_RASPBERRY32_PI) || defined(_RASPBERRY64_PI) || defined(_NVIDIA) || defined(__LINUX_RT_ARMV7__)

/*
** Open a video3d device. Pass a device identifier. If the device identifier is a number, such as "3", then it will
** open the device based on its index in the device list. The number may be octal or hexadecimal as well. Otherwise
** it will identify the device by serial number. A handle to the video3d device is returned in the given handle argument.
*/
EXTERN t_error
video3d_open(const char * device_id, t_video3d * handle);

/*
** Open a video3d file to emulate a device. Pass the path to the file. A handle to the video3d device is returned in
** the given handle argument.
*/
EXTERN t_error
video3d_open_file(const char * device_file, t_video3d * handle);

/*
** Enable a video3d stream.
** 
** The stream type indicates the type of image data to be enabled e.g. depth, infrared or color. The frame rate is in Hz.
** The image format and data type are defined in quanser_video.h.
**
** This function looks for an exact match for the stream type, frame width and frame height, but is more lenient
** with regards to the frame rate, finding the closest rate.
**
** This function returns a t_video3d_stream which may be used to change properties of the stream.
*/
EXTERN t_error
video3d_stream_open(t_video3d handle, t_video3d_stream_type type, t_uint index, t_double frame_rate, t_uint frame_width, t_uint frame_height, t_image_format format, t_image_data_type data_type, t_video3d_stream * stream);

/*
** Sets the value of a property of a stream. For boolean options use a value of 0.0 or 1.0.
** All other options should have a range of 0.0 to 1.0 inclusive. The function will do the
** appropriate scaling for the internal camera settings. Any values outside the range will
** be saturated to lie within the range.
*/
EXTERN t_error
video3d_stream_set_properties(t_video3d_stream stream, t_video3d_property * properties, size_t num_properties, t_double * values);

/*
** Get the camera intrinsics and distortion model. This function must be called *after* video3d_start_streaming or
** it will return an error on some platforms.
*/
EXTERN t_error
video3d_stream_get_camera_intrinsics(t_video3d_stream stream, t_single intrinsics[3][3], t_video3d_distortion_model* distortion, t_single distortion_coefficients[5]);

/*
** Get the homogeneous transformation from the "from_stream" to the "to_stream" extrinsics. The homogenous transformation
** takes the form:
**
**    R  d
**    0  1
**
** where R is a 3x3 rotation matrix and d is a 3-element translation vector. The elements are in MATLAB array order
** (column by column in memory).
*/
EXTERN t_error
video3d_stream_get_extrinsics(t_video3d_stream from_stream, t_video3d_stream to_stream, t_single transform[4][4]);

/*
** Get the depth scale for the given depth stream. If the stream is not a depth stream then zero is returned
** as the depth scale. The internal depth scale is adjusted to account for the image data type in which depth
** is represented.
*/
EXTERN t_error
video3d_stream_get_depth_scale(t_video3d_stream stream, t_image_data_type data_type, t_single* depth_scale);

/*
** Start streaming on all open streams.
*/
EXTERN t_error
video3d_start_streaming(t_video3d handle);

/*
** Get the latest frame for the given stream. If no frame is available then
** it returns -QERR_WOULD_BLOCK. Only one frame may be retrieved at a time
** and the frame must be released before getting a new frame.
*/
EXTERN t_error
video3d_stream_get_frame(t_video3d_stream stream, t_video3d_frame * frame);

/*
** Get the frame number.
*/
EXTERN t_error
video3d_frame_get_number(t_video3d_frame frame, t_uint64 * number);

/*
** Get the frame timestamp.
*/
EXTERN t_error
video3d_frame_get_timestamp(t_video3d_frame frame, t_double * timestamp);

/*
** Get the specifications of the frame passed to video3d_stream_open. Any parameter that is NULL will
** not be retrieved.
*/
EXTERN t_error
video3d_frame_get_image_specification(t_video3d_frame frame, t_double* frame_rate, t_uint* frame_width, t_uint* frame_height,
    t_image_format* format, t_image_data_type* data_type);

/*
** Get the frame data in the specified image format and data type. The data parameter must point to
** memory large enough to hold the image in the specified format and data type (HxWx3 or HxW).
*/
EXTERN t_error
video3d_frame_get_data(t_video3d_frame frame, void * data);

/*
** Get the frame motion data in the specified motion format. The data parameter must point to
** memory large enough to hold the motion data in the specified format.
*/
EXTERN t_error
video3d_frame_get_motion_data(t_video3d_frame frame, t_video3d_motion_format format, t_single * data);

/*
** Get the frame data in meters (only valid for depth streams). The data parameter must point to
** memory large enough to hold the image (HxW array of singles).
*/
EXTERN t_error
video3d_frame_get_meters(t_video3d_frame frame, t_single * data);

/*
** Release the frame so that it can be re-used by the video3d API.
*/
EXTERN t_error
video3d_frame_release(t_video3d_frame frame);

/*
** Stop all open streams.
*/
EXTERN t_error
video3d_stop_streaming(t_video3d handle);

/*
** Closes a stream.
*/
EXTERN t_error
video3d_stream_close(t_video3d_stream stream);

/*
** Closes a video3d device, release all its resources.
*/
EXTERN t_error
video3d_close(t_video3d handle);

#endif

#endif
