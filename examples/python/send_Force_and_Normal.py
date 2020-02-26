def send(forcepub, contacts, hands_list, feet_list, solution, logger) :

    forces_sheep = [solution.contact_values_map[feet_list[0]].force[0],
                    solution.contact_values_map[feet_list[0]].force[1],
                    solution.contact_values_map[feet_list[0]].force[2],
                    solution.contact_values_map[feet_list[1]].force[0],
                    solution.contact_values_map[feet_list[1]].force[1],
                    solution.contact_values_map[feet_list[1]].force[2],
                    solution.contact_values_map[hands_list[0]].force[0],
                    solution.contact_values_map[hands_list[0]].force[1],
                    solution.contact_values_map[hands_list[0]].force[2],
                    solution.contact_values_map[hands_list[1]].force[0],
                    solution.contact_values_map[hands_list[1]].force[1],
                    solution.contact_values_map[hands_list[1]].force[2]]


    normal_sheep = [solution.contact_values_map[feet_list[0]].normal[0],
                    solution.contact_values_map[feet_list[0]].normal[1],
                    solution.contact_values_map[feet_list[0]].normal[2],
                    solution.contact_values_map[feet_list[1]].normal[0],
                    solution.contact_values_map[feet_list[1]].normal[1],
                    solution.contact_values_map[feet_list[1]].normal[2],
                    solution.contact_values_map[hands_list[0]].normal[0],
                    solution.contact_values_map[hands_list[0]].normal[1],
                    solution.contact_values_map[hands_list[0]].normal[2],
                    solution.contact_values_map[hands_list[1]].normal[0],
                    solution.contact_values_map[hands_list[1]].normal[1],
                    solution.contact_values_map[hands_list[1]].normal[2]]


    print "contact_joints: ", contacts
    print "Sent forces_sheep is: ", forces_sheep
    print "Sent normals are: ", normal_sheep

    logger.add('contact_legs', forces_sheep)
    logger.add('contact_legs', normal_sheep)

    forcepub.sendForce(contacts, forces_sheep)
    forcepub.sendNormal(contacts, normal_sheep)