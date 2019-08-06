## Dockerfile 

To create a container with the dockerfile you need to have installed docker. Follow the instructions from this link: [docker in linux](https://runnable.com/docker/install-docker-on-linux). Once you have completed this step, you can proceed to build the container and run it:

```
docker build -t friendlyecho .
docker run -it friendlyecho bash
```