#!/bin/sh

set -e

# Docker login
echo $CI_REGISTRY_PASSWORD | docker login -u $CI_REGISTRY_USER --password-stdin $CI_REGISTRY

# Building container
docker build \
--network=host \
--build-arg ROS_DISTRO="melodic" \
--build-arg CI_REPOSITORY_URL="$CI_REPOSITORY_URL" \
--tag ci:build \
--file Dockerfile .
