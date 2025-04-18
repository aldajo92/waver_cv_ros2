#!/bin/bash

set -e

# Source the environment variables from config_docker.sh
PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"
source ${PROJECT_ROOT}/config_docker.sh

# Export the variables so docker-compose can use them
export DOCKER_IMAGE_NAME
export DOCKER_CONTAINER_NAME
export DOCKER_NETWORK
export DOCKER_REGISTRY_ADDR

echo "DOCKER_IMAGE_NAME: ${DOCKER_IMAGE_NAME}"
echo "DOCKER_CONTAINER_NAME: ${DOCKER_CONTAINER_NAME}"
echo "DOCKER_NETWORK: ${DOCKER_NETWORK}"
echo "DOCKER_REGISTRY_ADDR: ${DOCKER_REGISTRY_ADDR}"

# Down the containers
echo "executing the command: docker compose -f ${PROJECT_ROOT}/docker-compose.yml down"
docker compose -f ${PROJECT_ROOT}/docker-compose.yml down

# Pull the latest images
echo "executing the command: docker compose -f ${PROJECT_ROOT}/docker-compose.yml pull"
docker compose -f ${PROJECT_ROOT}/docker-compose.yml pull

# Restart the containers
echo "executing the command: docker compose -f ${PROJECT_ROOT}/docker-compose.yml up -d --force-recreate"
docker compose -f ${PROJECT_ROOT}/docker-compose.yml up -d --force-recreate --pull always

docker image prune -a -f
