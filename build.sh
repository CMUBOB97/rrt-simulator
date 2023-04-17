echo "Configuring and building..."

if [ ! -d "build" ]; then
  mkdir build
fi
cd build
cmake -DCMAKE_C_COMPILER=/opt/opencilk/bin/clang -DLLVM_CONFIG_PATH=/opt/opencilk/bin/llvm-config .. -DCMAKE_BUILD_TYPE=Release
make -j
