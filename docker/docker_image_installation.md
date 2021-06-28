# Docker image installation

The Docker image created with next steps is used for both IRI Summer School Workshops ([CNNS](https://gitlab.iri.upc.edu/mobile_robotics/summer_school/cnn_workshop/iri_traffic_sign_cnn_workshop_how_to) and [Navigation](https://gitlab.iri.upc.edu/mobile_robotics/summer_school/navigation_workshop/iri_summer_school_nav_workshop_how_to)). 

It's only needed to be followed once, and it will work for both workshops. 

## 1. Install docker

https://docs.docker.com/engine/install/

Only the following OS have been tested: Ubuntu 16.04, Ubuntu 18.04.

If your computer has a NVIDIA GPU, install nvidia-docker (2.0) to use Hardware Acceleration, otherwise you can skip this step:

https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker

Enable docker run without need of sudo:

```
sudo usermod -aG docker ${USER}
su - ${USER}  //or log out and log in
sudo systemctl restart docker
``` 

## 2. Create the docker image

* 2.1. Using a given Dockerfile.
* 2.2. Or downloading built image

### 2.1. Using a given Dockerfile

```
mkdir -p ~/summer_school/docker
cd ~/summer_school/docker
wget -O Dockerfile https://gitlab.iri.upc.edu/mobile_robotics/summer_school/cnn_workshop/iri_traffic_sign_cnn_workshop_how_to/-/raw/master/docker/Dockerfile
docker build -t summer_school .
```

After a while, if created successfully, it should appear in your docker images list:

```
docker images
```
Showing something like: 

```
REPOSITORY      TAG                    IMAGE ID       CREATED         SIZE
summer_school   latest                 ############   # minutes ago   4.51GB
```

### 2.2. Or download built image

Alternatively to creating the image, you can download the already built image [~4.6GB] from this [link](https://drive.google.com/file/d/1PmrnBD9nUjq-47OLSyzjw6adVkR2bFYW/view?usp=sharing)

And load it with:

```
docker load -i ~/Downloads/summer_school.tar
```

## 3. Additional resources

### Download the CNN data set

Download the reduced data set used to train the CNN from this IRI Google Drive [link](https://drive.google.com/file/d/1CpWXKGTJBopcYqFFgesFDFDUAkzgLmbK/view?usp=sharing).

Once downloaded in your host computer, from the download folder, move it to a shared folder that we will share with the docker container, and extract the contents. 

```
mkdir -p ~/summer_school/docker/shared
cp ~/Downloads/traffic_sign_data_set.tar.gz ~/summer_school/docker/shared/
cd ~/summer_school/docker/shared
tar -zxvf traffic_sign_data_set.tar.gz
```

## 4. Container initialization

### 4.1. Download script from PAL-robotics to be able to run Docker containers with GUI capabilities

```
cd ~/summer_school/docker
wget -O pal_docker.sh https://raw.githubusercontent.com/pal-robotics/pal_docker_utils/master/scripts/pal_docker.sh
chmod +x pal_docker.sh
```

### 4.2. Start container setting a shared folder

```
cd ~/summer_school/docker
./pal_docker.sh -it  --mount src="/home/$USER/summer_school/docker/shared",target="/root/shared",type=bind summer_school /bin/bash
```

#### 4.3. Copy the CNN data set

It is recommended to place it in the *~/summer_school* folder, but it can be placed anywhere. In that case, remember to change the path to the data set when launching the CNN training and testing scripts. 

```
#from inside the container
cp -r ~/shared/traffic_sign_data_set ~/summer_school/
```

### 5. Usage

To follow the workshop, from now on, if at any time you close all terminals in the container, the container state will be set to Exited.

To start the container again, assuming it's the last container used, for the first terminal, do:

```
docker start -a -i `docker ps -ql`
```

Then, to open new terminals on the same container, do:

```
docker exec -it `docker ps -ql` /bin/bash
```

**Note**: instead of    \``docker ps -ql`\` you can use a specific CONTAINER ID or NAME. Inspect them with `docker container list -a`.

