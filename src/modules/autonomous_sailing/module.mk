MODULE_COMMAND		= autonomous_sailing
SRCS				= autonomous_sailing.c \
                                        parameters.c \
                                        controller_data.c \
                                        guidance_module.c \
                                        send_msg_qgc.c  \
                                        mpcForces/mpc_boatTack_h20.c\
                                        mpcForces/mpc_boatTack_h30.c\
                                        mpc_test_data.c\
                                        extremum_sailcontrol.c

