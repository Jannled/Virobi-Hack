#Thanks to https://www.matansilver.com/2017/08/29/universal-makefile/

CC = gcc

TARGET_EXEC ?= Virobi-Hack
BUILD_DIR ?= bin
SRC_DIRS ?= src

#Thanks to https://stackoverflow.com/a/18258352
rwildcard=$(foreach d,$(wildcard $(1:=/*)),$(call rwildcard,$d,$2) $(filter $(subst *,%,$2),$d))

SRCS := $(call rwildcard,$(SRC_DIRS),*.cpp *.c)
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)

CFLAGS   = -I./src -I./lib -I./include -O0 -g3
CXXFLAGS = -std=c++14
LDFLAGS = -L./bin/openCV/lib -lopencv_world -Wl,-rpath=./bin/openCV/lib

#if shared library target
#CFLAGS += -shared -undefined dynamic_lookup

OPENCV_FOLDER = ./lib/opencv-4.2.0

ifeq ($(OS),Windows_NT)
	#Windows
	LDFLAGS += -lgdi32 -lopengl32
	OPENCV_DLL = $(BUILD_DIR)/openCV/lib/libopencv_world.dll
else
	#Linux
	LDFLAGS += -lX11 -lGL
	OPENCV_DLL = $(BUILD_DIR)/openCV/lib/libopencv_world.so
endif

all: $(BUILD_DIR)/$(TARGET_EXEC)

# main target (C++)
$(BUILD_DIR)/$(TARGET_EXEC): $(OBJS) opencv
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)

# c source
$(BUILD_DIR)/%.c.o: %.c
	$(MKDIR_P) $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# c++ source
$(BUILD_DIR)/%.cpp.o: %.cpp
	$(MKDIR_P) $(dir $@)
	$(CXX) $(CFLAGS) $(CXXFLAGS) -c $< -o $@

#Build OpenCV (Note: Under Linux you need to have libgtk2.0-dev and pkg-config installed)
opencv : $(OPENCV_DLL)

$(OPENCV_DLL):
	$(MKDIR_P) $(BUILD_DIR)/openCV/
	cd $(BUILD_DIR)/openCV/; \
	cmake "../../$(OPENCV_FOLDER)" -DCMAKE_RUNTIME_OUTPUT_DIRECTORY=$(BUILD_DIR) -DBUILD_opencv_world:BOOL="1" -DCMAKE_BUILD_TYPE=Release ; \
	make -j8
	$(foreach folder,$(patsubst($(wildcard $(OPENCV_FOLDER)/modules/*/include)),$(COPY) $(folder) ./;)
	$(COPY) $(OPENCV_FOLDER)/include/opencv2/* ./include/opencv2
	$(COPY) ./bin/openCV/opencv2/* ./include/opencv2

.PHONY: clean run all opencv

clean:
	$(RM) -r $(BUILD_DIR)

COPY ?= cp -R -L

MKDIR_P ?= mkdir -p

run: $(BIN)
	./$(BUILD_DIR)/$(TARGET_EXEC)