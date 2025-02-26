#!/bin/bash

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"
source ${PROJECT_ROOT}/config_docker.sh

docker buildx build --platform linux/arm64/v8 -t ${DOCKER_REGISTRY_ADDR}/${DOCKER_IMAGE_NAME} ${PROJECT_ROOT}
docker push ${DOCKER_REGISTRY_ADDR}/${DOCKER_IMAGE_NAME}
