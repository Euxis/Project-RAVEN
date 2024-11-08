## Getting Started

First, build the app files by using `./build.sh`

After you build, there should be a `bin` file

Unlike MOOS-IVP extend, the raw apps to be built are stored in:
```
../ivp/src/
```

### Can't build, no permission

This means you don't have execution permission for the file.

Run `chmod +x <filename>` to add execution permission to it. You might have to do the same for `build-moos.sh` and `build-ivp.sh`

### Can't build, "/bin/bash^M: bad interpreter: No such file or directory"

This means that the `build.sh` file was saved in Windows when you're using Unix. The easiest fix is to open the file in vim, with `vim build.sh` and typing `:set fileformat=unix` and pressing Enter.

You'll probably have to do the `build-moos.sh` and `build-ivp.sh` 


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

