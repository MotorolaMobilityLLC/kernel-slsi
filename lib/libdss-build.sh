#!/bin/bash
if [[ ${CC} = *"clang" ]]; then
${CC} \
  --target=aarch64-linux-gnu \
  -Ilib/libdss-include \
  -Iinclude \
  -I$1/include \
  -mlittle-endian \
  -Qunused-arguments \
  -fno-strict-aliasing \
  -fno-common \
  -fshort-wchar \
  -std=gnu89 \
  -DDSS_ANALYZER \
  -fno-PIE \
  -mno-implicit-float \
  -DCONFIG_BROKEN_GAS_INST=1 \
  -fno-asynchronous-unwind-tables \
  -fno-pic \
  -Oz \
  -Wframe-larger-than=4096 \
  -fno-stack-protector \
  -mno-global-merge \
  -fno-omit-frame-pointer \
  -fno-optimize-sibling-calls \
  -c \
  -static \
  -fno-strict-overflow \
  -fno-merge-all-constants \
  -fno-stack-check \
  -g lib/libdss.c -o ${1}lib/libdss.o

${CROSS_COMPILE}ar -rc ${1}libdss.a ${1}lib/libdss.o
fi
