
C_SOURCES = \
src/main.c \
drivers/CANOpen/src/CANOpen.c \
drivers/CANOpen/src/CANOpen_NMT.c \
drivers/CANOpen/src/CANOpen_LSS.c \

TARGET = CANAutoAssign 

LIBS = \

OUTPUT_DIR = build

INLCUDES = \
-Iinc/ \
-Idrivers/CANOpen/inc/ \

CC = gcc

CFLAGS = $(INLCUDES)

LDFLAGS = \

TEST_DIR = tests

# test:
# 	make -C $(TEST_DIR)
# test_clean:
# 	make -C $(TEST_DIR) clean

$(TARGET): $(OUTPUT_DIR)/$(TARGET)

OBJ = $(addprefix $(OUTPUT_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

$(OUTPUT_DIR)/%.o: %.c Makefile | $(OUTPUT_DIR) 
	$(CC) -c $(CFLAGS) $< -o $@

$(OUTPUT_DIR)/$(TARGET): $(OBJ) Makefile
	$(CC) $(OBJ) $(LDFLAGS) -o $@

$(OUTPUT_DIR):
	mkdir $@

clean:
	rm -rf $(OUTPUT_DIR)