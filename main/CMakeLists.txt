# Embed the server root certificate into the final binary
idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "log2file.c" "DIO.c" "pump_control.c" "irrigation.c" "ds18b20.c" 
                    INCLUDE_DIRS "."
					EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem)

