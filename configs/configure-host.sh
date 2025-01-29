#!/bin/bash

set -e

LIMO_PLATFORM_DIR=${HOME}/limo_platform
LIMO_PLATFORM_REPO=https://github.com/LCAS/limo_platform.git
CONFIGS_DIR=${LIMO_PLATFORM_DIR}/configs
ENV_FILE=${CONFIGS_DIR}/.env

if [ -r ${LIMO_PLATFORM_DIR}/.git ]; then
    echo "LIMO platform already exists, updating"
    (cd ${LIMO_PLATFORM_DIR} && git pull)
else
    echo "Cloning LIMO platform"
    git clone ${LIMO_PLATFORM_REPO} ${LIMO_PLATFORM_DIR}
fi

function add_to_env_file {
    echo "ensuring $1=$2 is in .env"
    if grep -q "^$1=" ${ENV_FILE}; then
        echo "$1 already set."
    else
        echo "Setting $1"
        echo "$1=$2" >> ${ENV_FILE}
    fi
}

echo "configure .env file"
if [ -r ${ENV_FILE} ]; then
    echo ".env already exists."
else
    touch ${ENV_FILE}
fi

add_to_env_file "UID" "`id -u`"
add_to_env_file "GID" "`id -g`"
add_to_env_file "HOSTNAME" "`hostname`"
add_to_env_file "ROBOT_NAME" "`hostname`"
add_to_env_file "ICON_PATH" "${CONFIGS_DIR}/icons"
add_to_env_file "DOCKER_COMPOSE_FILE" "${CONFIGS_DIR}/docker-compose.yaml"
add_to_env_file "LIMO_PLATFORM_DIR" "${LIMO_PLATFORM_DIR}"
add_to_env_file "LIMO_PLATFORM_DIR" "${LIMO_PLATFORM_DIR}"

echo "configure Desktop shortcuts"
for file in ${CONFIGS_DIR}/*.desktop.in; do
    dest_file="${HOME}/Desktop/$(basename ${file} .in)"
    echo "Configuring ${dest_file} from ${file}"
    export ROBOT_NAME=`hostname`
    export ICON_PATH=${CONFIGS_DIR}/icons
    export DOCKER_COMPOSE_FILE=${CONFIGS_DIR}/docker-compose.yaml
    if test ! -f ${DOCKER_COMPOSE_FILE}; then
        echo "No docker-compose file found, creating one"
        exit 1
    fi
    envsubst < ${file} > ${dest_file}
done

echo "installing conky - if not already installed"
apt update -y
apt upgrade -y
apt install conky-all -y

echo "configure docker check script"
export DOCKER_CONTAINER_NAME=limo_platform-limo_drivers-1
envsubst < ${CONFIGS_DIR}/check_container.sh.in > ${HOME}/scripts/check_container.sh
chmod +x ${HOME}/scripts/check_container.sh
cp -f ${CONFIGS_DIR}/conkyrc ${HOME}/.conkyrc

echo "preloading the docker image"
docker compose -f ${DOCKER_COMPOSE_FILE} pull
docker compose -f ${DOCKER_COMPOSE_FILE} up -d

echo "pruning docker system"
docker system prune -af



#m4 ${CONFIGS_DIR}/docker-off.desktop.in > ${HOME}/Desktop/limo.desktop
