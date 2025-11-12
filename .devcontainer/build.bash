#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR=$(dirname "$0")

# 设置默认 tag
TAG="nav"

# 从外部传入的 IMAGE_REPO（格式：ghcr.io/user/repo 或 docker.io/user/repo）
IMAGE_REPO=${IMAGE_REPO:-nagisa1201/njust-ai-assignment}

# 组合完整镜像名
IMAGE="$IMAGE_REPO:$TAG"

# 获取工作流中的环境变量来决定是否支持 arm64
PLATFORMS="linux/amd64"  # 默认只支持 amd64 架构

if [[ "${BUILD_ARM64}" == "true" ]]; then
    PLATFORMS="$PLATFORMS,linux/arm64"   # 如果环境变量 BUILD_ARM64 为 true，则支持 arm64 架构
fi
# 如果传入 --github-action 参数
if [[ "$1" == "--github-action" ]]; then
    echo "构建并推送镜像: $IMAGE"
    docker buildx build \
    --platform $PLATFORMS \
    -t "$IMAGE" \
    -f "$SCRIPT_DIR/Dockerfile" \
    "$SCRIPT_DIR" \
    --push
else
    echo "本地构建 $IMAGE"
    docker build -t "$IMAGE" "$SCRIPT_DIR"
fi
