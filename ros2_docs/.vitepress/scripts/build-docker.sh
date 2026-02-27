#!/bin/bash

SHELL_FOLDER=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
ROOT_FOLDER=$(dirname $(dirname "$SHELL_FOLDER"))
echo ROOT_FOLDER=$ROOT_FOLDER

cd $ROOT_FOLDER

REGISTRY="ccr.ccs.tencentyun.com/agilebot"
IMG_NAME="doc-site"
VERSION="latest-ros"

FULL_IMG_NAME="${REGISTRY}/${IMG_NAME}:${VERSION}"

docker build -t "$FULL_IMG_NAME" .
docker push "$FULL_IMG_NAME"
docker rmi "$FULL_IMG_NAME"
