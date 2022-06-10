idf_component_register(SRCS "mesh_main.c"							
							"measurement_task"
							"bme280.c"
							"i2c_application.c"
							"i2c_driver.c"
                    INCLUDE_DIRS "." "include" )
