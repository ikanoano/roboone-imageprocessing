# roboone-imageprocessing

An image-processing moudle of [hakaiSAN/ROBO-KEN-Action-Plan](https://github.com/hakaiSAN/ROBO-KEN-Action-Plan), which has operated the arm robot '[Quinque](https://www.robo-one.com/rankings/view/1127)' participated in the 10th and 11th [ロボ剣](https://www.robo-one.com/kens/index/55) as Team FMSKF705.
This module detects opponent's Men, Do and Kote in 3D coordinates from images taken with SR300 the depth camera.
![anim](https://user-images.githubusercontent.com/20738329/111065073-1b067d80-84fb-11eb-9e45-ab935e1614dd.gif)


## Requirements
* gcc 8.0
* librelasense v2.26.0
* OpenCV 4.1.1
* boost 1.67.0
* Eigen 3.3.9
```
# groupadd plugdev
# usermod -aG plugdev $USER
```

## Build
### Config 1. Unit test. Use pseudo input; run without realsense
```
make imtest
```

### Config 2. Unit test. With realsense
```
make imtest CAMERA=1
```

### Config 3. Unit test. Evaluate prediction
```
make imtest CAMERA=1 EVAL=1
./eval.sh
```
or
```
make imtest EVAL=1
./eval.sh
```

### Config 4. Create library
```
make
```
Usually makefile in [hakaiSAN/ROBO-KEN-Action-Plan](https://github.com/hakaiSAN/ROBO-KEN-Action-Plan) should handle it.


## Run the unit test
Connect SR300 camera to your PC if needed.
Then
```
./imtest
```

## License
TBD
