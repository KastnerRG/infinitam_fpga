deviceType            = [CPU, CUDA, OPENCL] # OPENCL only for no colors, scene integration, raycasting
openclDeviceType      = [CPU, GPU, FPGA] # Only for deviceType == OPENCL
openclAlgo            = [Integrate, Raycast, Both, Combined] # Only for deviceType == OPENCL
useSwapping           = [0, 1]
useApproximateRaycast = [0, 1] # Disable the Raycast algorithm (replace it)
useBilateralFilter    = [0, 1]
modelSensorNoise      = [0] # Automatically 1 if trackerType == TRACKER_WICP
trackerType = [
        TRACKER_COLOR,
        TRACKER_ICP,
        TRACKER_REN,
        TRACKER_IMU,
        TRACKER_IMU_ONLY,
        TRACKER_WICP,
        TRACKER_IMU_INIT] # All the IMU trackers don't work very well
                          # TRACKER_COLOR requires color information (--> no OpenCL)
noHierarchyLevels                = [1, 2, 3, 4, 5] # Not for TRACKER_IMU_ONLY, necessarily 2 for TRACKER_REN
trackingRegime                   = [BOTH_ONLY, TRANSLATION_ONLY, ROTATION_ONLY, BOTH_ROTATION, BOTH_TRANSLATION] # Not for TRACKER_IMU_ONLY
noICPIterationsInit              = [2, 4, 8, 10, 12]
skipPoints                       = [0, 1] # Only for TRACKER_COLOR
depthTrackerICPThreshold         = [0.01, 0.001, 0.005, 0.05, 0.1] # Only for TRACKER_ICP, TRACKER_IMU, TRACKER_WICP, TRACKER_IMU_INIT
depthTrackerTerminationThreshold = [0.001, 0.0001, 0.0005, 0.005, 0.01] # Only for TRACKER_ICP, TRACKER_IMU, TRACKER_WICP, TRACKER_IMU_INIT
voxelSize                        = [0.01, 0.001, 0.005, 0.02, 0.05, 0.1]
mu                               = [0.02, 0.01, 0.05, 0.1, 0.2]
stopIntegratingAtMaxW            = [0, 1]
maxW                             = [100, 10, 50] # Only if stopIntegratingAtMaxW == 1
