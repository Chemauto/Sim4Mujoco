#!/bin/bash
# Xbox手柄模拟器启动脚本

echo "================================"
echo "  Xbox手柄模拟器启动中..."
echo "================================"
echo ""

# 检查Python版本
python_version=$(python3 --version 2>&1 | awk '{print $2}')
echo "检测到Python版本: $python_version"

# 检查PyQt5是否安装
echo "检查依赖..."
python3 -c "import PyQt5" 2>/dev/null
if [ $? -ne 0 ]; then
    echo ""
    echo "⚠️  PyQt5未安装！"
    echo "请运行以下命令安装依赖："
    echo "  pip install -r requirements.txt"
    echo ""
    exit 1
fi

echo "依赖检查完成！"
echo ""
echo "启动手柄模拟器..."
echo "================================"
echo ""

# 启动程序
cd "$(dirname "$0")"
python3 src/main.py
