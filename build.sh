echo "Configuring and building..."

if [ ! -d "build" ]; then
  mkdir build
fi
cd build
cmake  .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-pg -DCMAKE_EXE_LINKER_FLAGS=-pg -DCMAKE_SHARED_LINKER_FLAGS=-pg
make -j

# -DCMAKE_C_COMPILER=/opt/opencilk/bin/clang -DLLVM_CONFIG_PATH=/opt/opencilk/bin/llvm-config
