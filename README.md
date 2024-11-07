## Getting Started

First, build the app files by using `./build`

After you build, there should be a `bin` file

Unlike MOOS-IVP extend, the raw apps to be built are stored in:
```
moos-ivp/ivp/src/
```


## Augmenting the $PATH to include apps

### Edit the .bashrc script

Go to `Ubuntu/home/` and open `.bashrc` with any text editor of your choosing.

At the end of the file, type in:
```
export PATH=$PATH:"/home/<name>/../Project-RAVEN/bin"
```
and
```
export PATH=$PATH:"/home/<name>/../Project-RAVEN/ivp/scripts/"
```

**DO NOT FORGET TO PUT `$PATH` OR ELSE YOUR PATH VARIABLE WILL BE CLEARED**

If this does happen, remove the line that changes the path variables and just restart the terminal 

### Non-permanent method

You can also just run the `export` commands straight into the terminal, but they will be erased after the next log in.

## Adding the program to CMakeLists.txt

This makes the program file staged to be built on the next ./build.sh

In `ivp/src`, open `CMakeLists.txt` and scroll down to `Part 7: Set Library/App categories and membership`

Add the app according to its use (its not a strict requirement, just for organization)

![Pasted image 20241104182510](https://github.com/user-attachments/assets/3948c9b1-f631-48f4-ba6c-645e76673f0f)

Here, I added `pOdometry` to `ROBOT_APPS` at the very bottom.

