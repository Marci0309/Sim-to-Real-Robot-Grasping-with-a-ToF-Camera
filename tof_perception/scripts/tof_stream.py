#!/usr/bin/env python3

import time
import cv2
import depthai as dai
import numpy as np

print(dai.__version__)

cvColorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
cvColorMap[0] = [0, 0, 0]

def create_pipeline():
    pipeline = dai.Pipeline()

    cam_tof = pipeline.create(dai.node.Camera)
    # fps set via pipeline
    cam_tof.setBoardSocket(dai.CameraBoardSocket.CAM_A)

    tof = pipeline.create(dai.node.ToF)
    cam_tof.raw.link(tof.input)

    # v3: use getInitialConfig() instead of initialConfig.get()
    tofConfig = tof.getInitialConfig()
    tofConfig.enableOpticalCorrection = True
    tofConfig.enablePhaseShuffleTemporalFilter = True
    tofConfig.phaseUnwrappingLevel = 4
    tofConfig.phaseUnwrapErrorThreshold = 300
    tofConfig.enableTemperatureCorrection = False
    tof.initialConfig.set(tofConfig)

    # v3: XLinkIn for runtime config updates
    xinTofConfig = pipeline.create(dai.node.XLinkIn)
    xinTofConfig.setStreamName("tofConfig")
    xinTofConfig.out.link(tof.inputConfig)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("depth")
    tof.depth.link(xout.input)

    return pipeline, tofConfig


if __name__ == '__main__':
    pipeline, tofConfig = create_pipeline()

    # v3: pass pipeline directly to Device, specify IP for PoE
    with dai.Device(pipeline, dai.DeviceInfo("169.254.1.222")) as device:
        print('Connected cameras:', device.getConnectedCameraFeatures())

        qDepth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        tofConfigInQueue = device.getInputQueue("tofConfig")

        print("Controls: f=FPN correction, o=optical, w=wiggle, t=temperature")
        print("          0-5=phase unwrapping level, m=median filter, q=quit")

        counter = 0
        while True:
            key = cv2.waitKey(1)
            if key == ord('f'):
                tofConfig.enableFPPNCorrection = not tofConfig.enableFPPNCorrection
                tofConfigInQueue.send(tofConfig)
            elif key == ord('o'):
                tofConfig.enableOpticalCorrection = not tofConfig.enableOpticalCorrection
                tofConfigInQueue.send(tofConfig)
            elif key == ord('w'):
                tofConfig.enableWiggleCorrection = not tofConfig.enableWiggleCorrection
                tofConfigInQueue.send(tofConfig)
            elif key == ord('t'):
                tofConfig.enableTemperatureCorrection = not tofConfig.enableTemperatureCorrection
                tofConfigInQueue.send(tofConfig)
            elif key == ord('q'):
                break
            elif key == ord('0'):
                tofConfig.enablePhaseUnwrapping = False
                tofConfig.phaseUnwrappingLevel = 0
                tofConfigInQueue.send(tofConfig)
            elif key in [ord('1'), ord('2'), ord('3'), ord('4'), ord('5')]:
                level = key - ord('0')
                tofConfig.enablePhaseUnwrapping = True
                tofConfig.phaseUnwrappingLevel = level
                tofConfigInQueue.send(tofConfig)
            elif key == ord('m'):
                medianSettings = [
                    dai.MedianFilter.MEDIAN_OFF,
                    dai.MedianFilter.KERNEL_3x3,
                    dai.MedianFilter.KERNEL_5x5,
                    dai.MedianFilter.KERNEL_7x7
                ]
                currentMedian = tofConfig.median
                nextMedian = medianSettings[(medianSettings.index(currentMedian) + 1) % len(medianSettings)]
                print(f"Median: {currentMedian.name} -> {nextMedian.name}")
                tofConfig.median = nextMedian
                tofConfigInQueue.send(tofConfig)

            imgFrame = qDepth.tryGet()
            if imgFrame is not None:
                depth_map = imgFrame.getFrame()
                max_depth = (tofConfig.phaseUnwrappingLevel + 1) * 1500
                depth_colorized = np.interp(depth_map, (0, max_depth), (0, 255)).astype(np.uint8)
                depth_colorized = cv2.applyColorMap(depth_colorized, cvColorMap)
                # Also show raw depth values in mm
                cv2.putText(depth_colorized,
                    f"Max range: {max_depth}mm | Press q to quit",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
                cv2.imshow("ToF Depth (OAK-D SR PoE)", depth_colorized)
                counter += 1
