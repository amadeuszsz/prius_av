docker build --network=host --no-cache \
	--build-arg ROS_DISTRO=noetic \
	--build-arg CUDA_IMAGE_TAG=11.4.1-devel-ubuntu20.04 \
	--build-arg CUDNN_VERSION=8.2.4.15-1+cuda11.4 \
	--build-arg TENSORRT_VERSION=8.2.4-1+cuda11.4 \
	-t amadeuszsz/prius_av:master .
