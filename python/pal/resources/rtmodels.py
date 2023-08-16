import os


__rtModelDirPath = os.path.normpath(
    os.path.join(os.environ['QAL_DIR'], 'libraries/resources/rt_models/')
)


# QCar RT Models
QCAR = os.path.normpath(
    os.path.join(__rtModelDirPath, 'qcar/QCar_Workspace'))

QCAR_STUDIO = os.path.normpath(
    os.path.join(__rtModelDirPath, 'qcar/QCar_Workspace_Studio'))

