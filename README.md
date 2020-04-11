# roboone-imageprocessing

## Requirements
* gcc 7.0
* librelasense v2.26.0
* OpenCV 4.1.1
* boost 1.67.0
```
# groupadd plugdev
# usermod -aG plugdev $USER
```

## Build
### Use pseudo input; run without realsense
```
make imtest
```

### With realsense
```
make imtest CAMERA=1
```

### evaluate prediction
```
make imtest EVAL=1
```
```
make imtest CAMERA=1 EVAL=1
```

## Run
```
./imtest
```


