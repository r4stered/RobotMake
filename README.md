# TODO:

## testing
test all configs on mac, linux, and windows

## utils
deploy script

## libs to add: 
phoenix 5
xrp vendordep
romi vendordep
msvc runtime?
libglass
libglassnt
halsim_ds_socket
halsim_gui
halsim_wsclient
halsim_wscore
halsim_wsserver
halsim_xrp

## Setup
1. Install python
2. Setup virtual env in root project folder: `python -m venv ./venv`
3. Activate virtual env: `./venv/Scripts/Activate.ps1`
4. Install required packages: `pip install -r requirements.txt`
5. To setup robot code:
   - To run on rio: `cmake --preset rio_release_shared`
   - To run on desktop:  `cmake --preset desktop_debug_shared`

## Building
1. `cd build/rio_release_shared/`
2. `cmake --build .`

## Deploying to robot
1. `python ./deploy.py`
