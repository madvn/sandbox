
## Setup
```console
foo@bar:~$ conda create -n tf-gpu tensorflow-gpu=1.15 python=2.7
foo@bar:~$ conda activate tf-gpu
foo@bar:~$ pip install -r requirements.txt
```

## Usage 

### Manually stitching clouds
```console
foo@bar:~$ conda activate tf-gpu
foo@bar:~$ cd src
foo@bar:~$ python stitcher.py /path/to/dataset/dir
```

### Axis calibration
```console
foo@bar:~$ conda activate tf-gpu
foo@bar:~$ cd src
foo@bar:~$ python calibration_optimizer.py --help
usage: calibration_optimizer.py [-h] [-d BASE_DIR] [-o OUTPUT_DIR]
                                [-e NUM_EPOCHS] [-v VIZ]

optional arguments:
  -h, --help            show this help message and exit
  -d BASE_DIR, --data-dir BASE_DIR [str, Required]
                        path to dataset
  -o OUTPUT_DIR, --output-dir OUTPUT_DIR [str, optional, default="./"]
                        path to store results in
  -e NUM_EPOCHS, --num-epochs NUM_EPOCHS [int, optional, default=100]
                        number of epochs
  -v VIZ, --visualize VIZ [int, optional, default=1]
                        flag to show results

```
