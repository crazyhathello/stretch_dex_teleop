####################################################
# Utility code to parse coordinates from 

import csv

def read_controller_raw_data():
    controller_input = "translation_testing_coordinates.tsv"

    right_x = []; right_y = []; right_z = []
    right_rotation = []

    with open(controller_input, 'r') as f:
        reader = csv.reader(f, delimiter='\t')
        for row in reader:
            # Right Rotation
            cur_right_rotation = row[3]
            ## Skip if rotation value is 0
            if cur_right_rotation != '0,0,0,1':
                right_rotation.append(cur_right_rotation)
            
            # Right XYZ
            right_xyx = row[1].split(',')
            ## Skip is xyz value is 0
            if float(right_xyx[0]) != 0.0 and float(right_xyx[1]) != 0.0 and float(right_xyx[2]) != 0.0:
                right_x.append(float(right_xyx[0]))
                right_y.append(float(right_xyx[1]))
                right_z.append(float(right_xyx[2]))
    return right_x, right_y, right_z

def get_controller_data_y():
    right_x, right_y, right_z = read_controller_raw_data()
    base_y = right_y[0]
    right_y_rel = []

    # Get the delta value
    for item in right_y[1:]:
        right_y_rel.append(item - base_y)
    return right_y_rel

def get_controller_data_z():
    right_x, right_y, right_z = read_controller_raw_data()
    base_z = right_z[0]
    right_z_rel = []

    # Get the delta value
    for item in right_z[1:]:
        right_z_rel.append(item - base_z)
    return right_z_rel

def get_controller_data_all():
    right_x, right_y, right_z = read_controller_raw_data()
    base_y = right_y[0]; base_z = right_z[0]
    right_yz_rel = []

    # Get the delta value
    for i in range(1, len(right_y)):
        cur_y = right_y[i] - base_y
        cur_z = right_z[i] - base_z
        right_yz_rel.append((cur_y, cur_z))
    return right_yz_rel

    
    '''print(len(right_rotation))
    print(f'Min of right_x_rel: {min(right_x_rel)}')
    print(f'Max of right_x_rel: {max(right_x_rel)}')
    print(f'Min of right_y_rel: {min(right_y_rel)}')
    print(f'Max of right_y_rel: {max(right_y_rel)}')
    print(f'Min of right_z_rel: {min(right_z_rel)}')
    print(f'Max of right_z_rel: {max(right_z_rel)}')'''