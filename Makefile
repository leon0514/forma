GREEN  := \033[0;32m
BLUE   := \033[0;34m
YELLOW := \033[0;33m
CYAN   := \033[0;36m
RESET  := \033[0m # 用于重置颜色，非常重要！

# 编译器和基本配置
cc          := g++
stdcpp      := c++17

# 目标文件
target_exe  := pro
target_lib  := libforma.so

# 目录定义
workdir     := workspace
srcdir      := src
objdir      := objs

# 依赖库路径
ascend_home := /home/HwHiAiUser/Ascend/ascend-toolkit/latest
opencv_home := /home/HwHiAiUser/project/__install/opencv455

# --- 编译和链接标志 ---

# 预处理器定义
DEFS := -DENABLE_DVPP_INTERFACE

# 头文件搜索路径 (-I)
include_paths := \
    src \
    $(opencv_home)/include/opencv4 \
    $(ascend_home)/thirdpart/include/acllite \
    $(ascend_home)/aarch64-linux/include

# 库文件搜索路径 (-L)
library_paths := \
    $(opencv_home)/lib \
    $(opencv_home)/3rdparty/lib \
    $(ascend_home)/aarch64-linux/lib64/ \
    $(ascend_home)/runtime/lib64/stub \
    $(ascend_home)/thirdpart/lib

# 需要链接的库 (-l)
link_opencv       := opencv_highgui opencv_videoio opencv_imgproc opencv_imgcodecs opencv_core
link_3rdparty     := ade ittnotify libjpeg-turbo libopenjp2 libpng libprotobuf libtiff libwebp tegra_hal IlmImf zlib
link_ascend       := ascendcl acl_dvpp acllite
link_sys          := stdc++ dl rt
link_librarys     := $(link_opencv) $(link_3rdparty) $(link_sys)

# --- 自动生成编译和链接选项 ---

# 将路径和库名转换为编译器可用的标志
INCLUDE_FLAGS := $(foreach item,$(include_paths),-I$(item))
LIBRARY_FLAGS := $(foreach item,$(library_paths),-L$(item))
LINK_LIB_FLAGS:= $(foreach item,$(link_librarys),-l$(item))
RPATH_FLAGS   := $(foreach item,$(library_paths),-Wl,-rpath=$(item))

# 最终的编译和链接标志
CPP_COMPILE_FLAGS := -std=$(stdcpp) -w -g -O0 -fPIC -fopenmp -pthread $(INCLUDE_FLAGS) $(DEFS)

# add  -fsanitize=address -fno-omit-frame-pointer for debug
LINK_FLAGS := -pthread -fopenmp -Wl,-rpath='$$ORIGIN' \
              -Wl,--no-as-needed $(LIBRARY_FLAGS) $(LINK_LIB_FLAGS) \
              -Wl,--as-needed $(RPATH_FLAGS) 

# --- 源文件和目标文件处理 ---

# 查找所有.cpp源文件
all_srcs := $(shell find $(srcdir) -name "*.cpp")

# 筛选出用于不同库的源文件
# 假设 threadInfer 目录下的文件用于 infer_lib, 其他所有文件用于 shared_lib 和 pro
infer_srcs := $(filter %threadInfer%, $(all_srcs))
# pipe_srcs 是除 infer_srcs 之外的所有源文件 (如果需要分别编译)
# cpp_srcs 现在代表所有源文件，用于构建 pro 和 shared_lib
cpp_srcs := $(all_srcs) 

# 根据源文件生成对象文件列表，并指定输出到 objdir
# 使用标准 .o 后缀
cpp_objs   := $(cpp_srcs:$(srcdir)/%.cpp=$(objdir)/%.o)
infer_objs := $(infer_srcs:$(srcdir)/%.cpp=$(objdir)/%.o)

# 根据对象文件生成依赖文件列表 (.d)
dep_files := $(cpp_objs:.o=.d)

# 如果不是 clean 命令，则包含所有依赖文件
ifneq ($(MAKECMDGOALS), clean)
-include $(dep_files)
endif

# --- 伪目标和构建规则 ---

.PHONY: all pro lib run clean help

# 默认目标
all: $(workdir)/$(target_exe) $(workdir)/$(target_lib)

# 别名目标
pro:    $(workdir)/$(target_exe)
lib:  $(workdir)/$(target_lib)


# 运行目标
run: $(workdir)/$(target_exe)
	@echo "--- Running $(target_exe) ---"
	@cd $(workdir) && ./$(target_exe)

# --- 链接规则 ---

# 链接可执行文件 pro
$(workdir)/$(target_exe): $(cpp_objs)
	@echo "$(YELLOW)==> Linking Executable: $@$(RESET)"
	@mkdir -p $(dir $@)
	@$(cc) $^ -o $@ $(LINK_FLAGS)

# 链接共享库 libascendpipe.so
# 注意: 通常可执行文件和共享库不会使用完全相同的对象文件，这里假设是这样
$(workdir)/$(target_lib): $(cpp_objs)
	@echo "$(YELLOW)==> Linking Shared Library: $@$(RESET)"
	@mkdir -p $(dir $@)
	@$(cc) -shared $^ -o $@ $(LINK_FLAGS)


# --- 编译和依赖生成规则 ---

# 编译 .cpp 文件为 .o 文件 (通用规则)
# 此规则可以处理所有 src/ 目录下的 .cpp 文件，并将其 .o 文件输出到 objdir/
$(objdir)/%.o: $(srcdir)/%.cpp
	@echo "$(GREEN)==> Compiling CXX: $<$(RESET)"
	@mkdir -p $(dir $@)
	@$(cc) $(CPP_COMPILE_FLAGS) -c $< -o $@

# 生成依赖文件 .d
$(objdir)/%.d: $(srcdir)/%.cpp
	@echo "$(BLUE)==> Generating Dependencies for: $<$(RESET)"
	@mkdir -p $(dir $@)
	@$(cc) -M $(CPP_COMPILE_FLAGS) $< | sed 's,\($*\)\.o[ :]*,\1.o \1.d : ,g' > $@


# --- help ---
help:
	@echo "Usage: make [target]"
	@echo ""
	@echo "Targets:"
	@echo "  all      Build all targets (default)"
	@echo "  pro      Build the executable ($(target_exe))"
	@echo "  run      Run the executable"
	@echo "  clean    Remove build artifacts"
	@echo "  help     Show this help message"

# --- 清理规则 ---
clean:
	@echo "==> Cleaning workspace..."
	@rm -rf $(objdir) $(workdir)/$(target_exe) $(workdir)/$(target_lib)