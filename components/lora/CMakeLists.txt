#set(COMPONENT_ADD_INCLUDEDIRS csrc)
#set(COMPONENT_SRCS "csrc/lora.c")
#set(COMPONENT_NAME ".")
#register_component()

idf_component_register(
        SRCS csrc/lora.c
        INCLUDE_DIRS csrc
	REQUIRES driver
    )