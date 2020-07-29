### Step 1: Install Docker

###### Setup the repository

1. Update the `apt` package index and install packages to allow `apt` to use a repository over HTTPS:

   ```
   $ sudo apt-get update
   
   $ sudo apt-get install \
       apt-transport-https \
       ca-certificates \
       curl \
       gnupg-agent \
       software-properties-common
   ```

2. Add Docker’s official GPG key:

   ```
   $ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
   ```

   Verify that you now have the key with the fingerprint `9DC8 5822 9FC7 DD38 854A E2D8 8D81 803C 0EBF CD88`, by searching for the last 8 characters of the fingerprint.

   ```
   $ sudo apt-key fingerprint 0EBFCD88
   
   pub   rsa4096 2017-02-22 [SCEA]
         9DC8 5822 9FC7 DD38 854A  E2D8 8D81 803C 0EBF CD88
   uid           [ unknown] Docker Release (CE deb) <docker@docker.com>
   sub   rsa4096 2017-02-22 [S]
   ```

3. Use the following command to set up the **stable** repository. To add the **nightly** or **test** repository, add the word `nightly` or `test` (or both) after the word `stable` in the commands below. [Learn about **nightly** and **test** channels](https://docs.docker.com/engine/install/).

> **Note**: The `lsb_release -cs` sub-command below returns the name of your Ubuntu distribution, such as `xenial`. Sometimes, in a distribution like Linux Mint, you might need to change `$(lsb_release -cs)` to your parent Ubuntu distribution. For example, if you are using `Linux Mint Tessa`, you could use `bionic`. Docker does not offer any guarantees on untested and unsupported Ubuntu distributions.

x86_64 / amd64

```
$ sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"
```

###### Install Docker Engine

1. Update the `apt` package index, and install the *latest version* of Docker Engine and containerd, or go to the next step to install a specific version:

   ```
    $ sudo apt-get update
    $ sudo apt-get install docker-ce docker-ce-cli containerd.io
   ```

2. Verify that Docker Engine is installed correctly by running the `hello-world` image.

```
$ sudo docker run hello-world
```

This command downloads a test image and runs it in a container. When the container runs, it prints an informational message and exits.

[https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/)

### Step 2: Pull the Docker Image

Docker Pull Command　

```
docker pull yz16/my_ros
```

[https://hub.docker.com/r/yz16/my_ros](docker pull yz16/my_ros)

### Step 3: Docker Volume

###### Create a docker volume

```livescript
$ sudo docker volume create ros_workspace
```

###### List all the volume

```
$ docker volume ls

local               ros_workspace
```

###### Inspect information in the volume

```
$ docker volume inspect ros_workspace
[
    {
        "Driver": "local",
        "Labels": {},
        "Mountpoint": "/var/lib/docker/volumes/ros_workspace/_data",
        "Name": "ros_workspace",
        "Options": {},
        "Scope": "local"
    }
]
```

###### Run the docker mounted a volume

```
sudo docker run --name ros_env \
--mount type=bind,source=/var/lib/docker/volumes/ros_workspace,target=/root/ros_workspaces \
-itd -p 6080:80 yz16/my_ros
```

They you can use the IP+6080 to login the interface, like `10.20.4.70:6080`.

###### If you want to mount the folder to your define location

```livescript
sudo docker run --name ros_env_star -v /home/star/Desktop/ros_workspace:/root/ros_workspaces -itd -p 6080:80 yz16/my_ros
```

###### If you want to stop the container you can do 

```livescript
sudo docker stop ros_env
```

###### If you want to start the container you can do 

```livescript
sudo docker start ros_env
```

###### Also, you can check the status of container 

```livescript
sudo docker container ps
```

ros_env

ros_env_star

https://yeasy.gitbook.io/docker_practice/data_management/volume

[https://zhuanlan.zhihu.com/p/85664330](https://zhuanlan.zhihu.com/p/85664330)

###### The listed codes are the reference to checking the additional functions.

Run a docker and mount a volume 

```
$ docker run -d -P \
    --name web \
    # -v my-vol:/wepapp \
    --mount source=my-vol,target=/webapp \
    training/webapp \
    python app.py
```

Inspect a container

```
$ docker inspect web
```

```
"Mounts": [
    {
        "Type": "volume",
        "Name": "my-vol",
        "Source": "/var/lib/docker/volumes/my-vol/_data",
        "Destination": "/app",
        "Driver": "local",
        "Mode": "",
        "RW": true,
        "Propagation": ""
    }
],
```

Delete the volume

```
$ docker volume rm my-vol
```

If you want to delete the docker with volume, you can use the command like `docker rm -v`

Delete the volume without the container

```
$ docker volume prune
```

[https://yeasy.gitbook.io/docker_practice/data_management/volume](https://yeasy.gitbook.io/docker_practice/data_management/volume)



## Install ROS

###### Update the source list, USTc YUAN

```
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
```

[https://www.linuxidc.com/Linux/2019-05/158461.htm](https://www.linuxidc.com/Linux/2019-05/158461.htm)

###### A tool to check the IP of dynamic name domain 

`nslookup` 

```
151.101.229.194 github.global.ssl.fastly.net
13.250.177.223 github.com
```

```
sudo /etc/init.d/networking restart
```

###### MAKE Problem

the protoc version is different
[https://blog.csdn.net/tuhuolong/article/details/78353519](https://blog.csdn.net/tuhuolong/article/details/78353519)

I deleted two protoc, and Install protoc
[https://www.jianshu.com/p/ae41a4659e6d](https://www.jianshu.com/p/ae41a4659e6d)

[https://blog.csdn.net/qq_25368751/article/details/104248464](https://blog.csdn.net/qq_25368751/article/details/104248464)

###### rosdep Problem

[https://blog.csdn.net/qq_43310597/article/details/106034812](https://blog.csdn.net/qq_43310597/article/details/106034812)
配置rosdep时总是会出现这样那样的错误,本文只针对sudo rosdep init执行过程中最容易出现报错的情况进行讨论.
毕竟rosdep update 也是在sudo rosdep init 步骤成功后才可以执行.

常见的报错形式:
ERROR: cannot download default sources list from:
[https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list](https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list)
Website may be down.

这种情况其实不算是网络的问题,果然换了手机热点再执行也没用,只能说网站被墙了.那么要解决被墙,思路就是在host文件中添加网站的ip地址.

方法:`sudo gedit /etc/hosts`

打开host文件后,在文件的最末尾添加:`151.101.84.133  raw.githubusercontent.com`

保存并关闭文件,再执行sudo rosdep init,可以看到

```
Wrote /etc/ros/rosdep/sources.list.d/20-default.list
Recommended: please run
rosdep update
```

说明这一步成功,接下来执行rosdep update,慢慢等待,最终也成功.问题顺利解决.
[https://blog.csdn.net/qq_43310597/java/article/details/106034812](https://blog.csdn.net/qq_43310597/java/article/details/106034812)

### How to Use Git on Ubuntu

1. install the git

```
sudo apt-get install git
```

2. set up the information of github

```
git config --global user.name "your name here" 
git config --global user.name "your email"
```

3. set the credential helper

```
git config --global credential.helper cache 
git config --global credential.helper 'cache --timeout=3600' 
```

4. Initial an empty repository

```
git init
touch README
git add README
git commit -m "first commit"
```

5. submit to your repository

```
git remote add origin https://github.com//SurfaceSun//DailyCode.git
```

6. Submit command

```
git push -u origin master
```

7. If you want to clone an existed repository

```
git clone https://github.com/SurfaceSun/DailyCode.git
```

8. Modify the file and submit

```
git add .
git commit -m "Write down something"
git push -u origin master
```

[https://www.runoob.com/manual/git-guide/](https://www.runoob.com/manual/git-guide/)
