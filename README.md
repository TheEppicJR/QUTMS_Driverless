# QUTMS_Driverless

# Run FS Simulator - For Docker with WSL

## Set Up Repository

Clone/download this repository into a folder in Windows using any method of choice.


## Set Up WSL 
### Enable WSL
 
To enable WSL, open command prompt and type: 
> `dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart`

and:

> `dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart`

Restart your machine following this.
 
 
### Install Ubuntu
 
Install Ubuntu (or Ubuntu 20.04 LTS) from the Microsoft store and run it.
When prompted, complete account setup.
Additionally, run:
> `sudo apt update`

and:

> `sudo apt upgrade`

 
### Install Terminal
 
Install Windows terminal from the Microsoft store.
This allows access to the command line interface for WSL distributions.

 
### Install Windows XServer

Install VcXsrv (Windows XServer) from:
> https://sourceforge.net/projects/vcxsrv/

Run Xlaunch.
Leave all setting as default except for the following:

![Xlaunch Config](/images/xserver_setting.png)

Ensure "Native opengl" is unchecked and "Disable access control" is checked.
The XServer icon should appear on the taskbar tray with 0 clients so far.
 
 
## Set Up FS Simulator
 
Download FS Driverless Simulator from the Formula Student GitHub:

> https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.0.0/fsds-v2.0.0-win.zip

Extract the files to a folder of your choosing.

In the QUTMS Driverless repo folder `fs_sim_settings`, there is a folder titled: `Formula-Student-Driverless-Simulator`.
Copy the entire folder into your user directory. 
Eg. `C:\Users\Username\`.

To test this was successful, run `FSDS.exe`. 
You should be able to run the simulation and control the car with 'WASD' keys.
 
 
## Set Up Docker environment
 
Install Docker Desktop from:
> https://www.docker.com/products/docker-desktop


### Docker settings configuration

![Docker General Settings](/images/docker_settings1.png)

Ensure "Expose daemon on tcp://localhost:2375 without TLS" is unchecked.


![Docker WSL Settings](/images/docker_settings2.png)

To avoid conflicts with multiple distros, uncheck any "Enable integration with additional distros:".
(our environment is being built in windows anyway).


### Building Images

In the CMD terminal, type:

> `ipconfig`

and copy the value next to 'IPV4 Address'.

Rename the file `env` to `.env`. This allows environment variables to be passed from windows into the container.

Using a text editor of choice (recommend VS Code), open and edit the `.env` file included in the parent directory of the repo.
This file handles user specific data each time the image is run. - might be a better explanation later

Replace instances of `{IPV4-HERE}` with your copied IPV4 address. For example:

> `SIM_HOST=100.10.10.10`
>
> `DISPLAY=100.10.10.10:0.0`

Save and close the `.env` file.

In the Ubuntu terminal, navigate to the downloaded repository folder, or on the folder in file explorer, right click and select "Open in Terminal".

Once in the QUTMS Driverless directory, type:

> `make build`

If this returns an error saying make is not installed, install it with:

> `sudo apt install make`


Building the FS simulator, ros, and ros1-ros2 bridge images will take upwards of 10 minutes - possibly nearly an hour.
Your CPU will max out at 100% in parts and the most RAM usage experienced will be approximately 21GB - Don't panic if you don't have 21GB of ram, your machine will just get fairly slow once it hits this stage.

After the build has been completed once, most processes are cached for future builds if necessary (don't corrupt things and you wont have to rebuild).


### Building Ros2 in Container

To build a development container with the Docker image, navigate to the same QUTMS Driverless directory where you built the image and type:

> `make run_ros`

This will bring you into the virtual container made by the image. The command line will now look like this:

> `developer@docker-desktop:/home/developer/driverless_ws$`

To build the ros2 packages for the FS simulator, type:

> `colcon build`

First time building this may take up to 3 minutes, sequential builds inside the container may be quicker.

Now, to initialise scripts with the ros2 environment, type:

> `source install/setup.bash`


### Running Scripts

Open a new terminal tab and navigate to the same QUTMS Driverless directory where we have been working, or open the folder in terminal. Ensure XServer and the FSDS simulator is running. Type:

> `make run_sim_bridge`

This will make a 'bridge' between the FS simulator and the ros2 node where scripts are running so data can be output and read.


Finally, to run the included script that has been developed, type:

> `ros2 run simple_control main`

This would change depending on the name of the program, where simple_control is the folder within src, and main.cpp is the script.