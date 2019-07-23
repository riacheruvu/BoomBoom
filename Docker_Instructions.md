# Docker Instructions

## How do I perform Docker Development on my duckiebot?

You can choose from the following options:

- Developer Docker containers on your laptop and then deploy them directly to the robot
- Develop Docker containers directly on your laptop

Here's the few relevant paragraphs from the [DT18 instructions on "Duckiebot Development using Docker"](https://docs.duckietown.org/DT18/software_devel/out/autoid_DO_NOT_USE_THIS_VERY_UNSTABLE_LINK_5f3ae1fc_109.html):

![1563863211092](C:\Users\Turia\AppData\Roaming\Typora\typora-user-images\1563863211092.png)

## How do I create a docker container with an already provided base image?

#### Team-specific notes: We probably won't need to do this multiple times, since we already have a container to work with.

For the purposes of this project, we've chosen to build upon *rpi-duckiebot-base:master19* - this image contains most of the packages we need to complete this project.

To create a Docker container with this image, I ran something similar to the following command, where boomboom is the name of my container:

docker -H riabot.local run -dit --privileged --name boomboom --network=host -v /data:/data duckietown/rpi-duckiebot-base:master19

I then saw the container pop up in my Portainer.

## How do I pull/push images from DockerHub?

To push to the repository:

```
docker  push 369r963/dgmds14teamrepo:tagname
```

To pull from the repository:

```
docker  pull 369r963/dgmds14teamrepo:tagname
```

Check out [DockerHub's documentation on the topic](https://docs.docker.com/docker-hub/) for more information.

## What's the difference between running and starting a Docker container, and what are the commands?

When setting up our duckiebot and software, we previously used commands like the following (taken from ["Unit C-6 Setting up the Dashboard"](http://docs.duckietown.org/DT19/opmanual_duckiebot/out/duckiebot_dashboard_setup.html) of the DT19 instructions):

```
docker -H *hostname*.local run -dit --net=host --name dashboard duckietown/rpi-duckiebot-dashboard:master19
```

Let's say you try and pull an image from our repository **on your laptop**. If you execute the below command, you will see the new container and image you pulled in in your Portainer:

```
docker -H riabot.local run -dit --net=host --name try_cont 369r963/dgmds14teamrepo:latest
```

The command will promptly execute. Now that the container's created, we're good to go - but we need to be able to reuse it. If you try the same command again, you'll see the error:

```
docker: Error response from daemon: Conflict. The container name "/try_cont" is already in use by container "7996aaad78afdf884484a201b2809e1acbe9dec6196bcd199c171dd3959ee3f8". You have to remove (or rename) that container to be able to reuse that name.
```

Here, we come to [**the difference between running and starting a container**](<https://stackoverflow.com/questions/34782678/difference-between-running-and-starting-a-docker-container>):

- Running involves creating and executing a new container of the image
- Starting launches a container that was previously stopped - retaining the data and the settings.

If you want to execute files directly on the container, you can just run the following command. --interactive attaches the container's STDIN, so you can now access files within the container.

```
docker -H riabot.local start try_cont --interactive
```

**Alternatively, you could just ssh into your robot, exclude the *"-H hostname.local"* parts, and execute the docker commands, and it'll work the same.**

