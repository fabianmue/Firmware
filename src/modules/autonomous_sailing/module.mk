MODULE_COMMAND		= autonomous_sailing
SRCS				= autonomous_sailing.c \
					navigation.c \
                                        parameters.c \
                                        controller_data.c \
                                        guidance_module.c \
                                        path_planning.c \
                                        hil_simulation.c \
                                        simulation_utility.c \
                                        reference_actions.c \
                                        send_msg_qgc.c
					
OBJS += mpc_boatTack.o
