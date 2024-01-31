set(
	SOURCES
	ultrasound.c
	utils.c
)


idf_component_register(
	SRCS ${SOURCES}
	INCLUDE_DIRS "include"
	REQUIRES freertos hal driver esp_timer xtensa
)
