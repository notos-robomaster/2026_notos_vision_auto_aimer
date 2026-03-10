#!/bin/bash
# Jetson NX 编译脚本
# 用法: ./build_jetson.sh

set -e  # 遇到错误立即退出

echo "=========================================="
echo "   Jetson NX 自瞄系统编译脚本"
echo "=========================================="

# 检测系统架构
ARCH=$(uname -m)
echo "检测到系统架构: $ARCH"

if [[ "$ARCH" != "aarch64" && "$ARCH" != "arm64" ]]; then
    echo "警告: 当前系统不是 ARM64 架构，但仍会继续编译"
fi

# 检查 CUDA
if command -v nvcc &> /dev/null; then
    CUDA_VERSION=$(nvcc --version | grep "release" | sed 's/.*release //' | sed 's/,.*//')
    echo "✓ CUDA 已安装: $CUDA_VERSION"
else
    echo "⚠ CUDA 未检测到，性能可能受限"
fi

# 检查 OpenCV
if pkg-config --exists opencv4; then
    OPENCV_VERSION=$(pkg-config --modversion opencv4)
    echo "✓ OpenCV 已安装: $OPENCV_VERSION"
    
    # 检查 CUDA 支持
    if pkg-config --variable=OPENCV_CUDA opencv4 | grep -q "YES"; then
        echo "✓ OpenCV CUDA 支持已启用"
    else
        echo "⚠ OpenCV 未启用 CUDA 支持，建议重新编译 OpenCV"
    fi
else
    echo "✗ OpenCV 未找到，请先安装"
    exit 1
fi

# 清理旧的构建
echo ""
echo "清理旧的构建文件..."
rm -rf build/
mkdir -p build

# 配置 CMake
echo ""
echo "配置 CMake..."
cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17

# 编译
echo ""
echo "开始编译 (使用 $(nproc) 个核心)..."
make -C build/ -j$(nproc)

# 检查编译结果
if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "   ✓ 编译成功！"
    echo "=========================================="
    echo ""
    echo "可执行文件位于 build/ 目录："
    ls -lh build/auto_aim_test build/camera_test 2>/dev/null || echo "部分可执行文件生成失败"
    echo ""
    echo "运行测试:"
    echo "  ./build/auto_aim_test -c configs/demo.yaml"
    echo ""
else
    echo ""
    echo "=========================================="
    echo "   ✗ 编译失败"
    echo "=========================================="
    exit 1
fi
