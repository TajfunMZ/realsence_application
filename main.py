import numpy as np
from basic import loadCalibration, startMeasurment
from cameraF import initCamera, loadPCD
from mathF import average
from calibrate import captureReference
from optimise import optimiseWithAutofloorSize, findOptimalParameters
from datetime import datetime
import sys, copy, csv, time, os, xlsxwriter


if __name__ == '__main__':   
    no_arguments = len(sys.argv)
    acceptableArgList = ['calibrate', 'measure', 'help', 'optimise', 'optimise2', 'measure_xls']

    if(no_arguments >= 2):
        
        # start camera
        pipe = initCamera(True)
        mode = sys.argv[1]
        if mode in acceptableArgList:
            print("\nRunning...")
            if(mode == 'calibrate'):
                if(no_arguments >= 3):
                    savefile = sys.argv[2]
                    zero_volume = int(sys.argv[3]) if no_arguments >= 4 else -1
                    auto_detect = bool(sys.argv[4]) if no_arguments >= 5 else False
                    save_pcd = bool(sys.argv[5]) if no_arguments >= 6 else False
                    captureReference(savefile, pipe, zero_volume, True, auto_detect, 10, save_pcd)
                else:
                    print('Please also provide a name for the new configuration file without the ".json" sufix.')

            elif(mode == 'measure'):
                if(no_arguments >= 3):
                    
                    # get the wanted number of measurments
                    file_name = sys.argv[2]
                    no_iterrations = int(sys.argv[3]) if no_arguments >= 4 else 10
                    name_extention = sys.argv[4] if no_arguments >=5 else ''

                    if(os.path.isfile('./configuration_files/' + file_name + '.json')):
                        # Load point coordinates and pcd rotation matrix
                        calibration_json = loadCalibration(file_name)
                    else:
                        # If no file is present calibrate and load
                        print('Calibration file not found. Calibrating...')
                        captureReference(file_name, pipe, -1, True, True)
                        input('\nEnter anything to continuing collecting measurments.')

                        # Load point coordinates and pcd rotation matrix
                        calibration_json = loadCalibration(file_name)
                    
                    milis_since_epoch = round(time.time()*1000)
                    save_file = file_name + '_' + name_extention + '_' + str(milis_since_epoch) + '.csv'
                    # Open results csv and add time in miliseconds to not overwrite existing results
                    with open("./csv_results/" + save_file, 'w', newline='') as file:
                        writer = csv.writer(file)
                        writer.writerow(["Iterration number", "Volume [m^3]", "Volume difference [m^3]", "Time"])

                        # get measurments with a pause between each picture
                        for measurment in range(no_iterrations):
                            measured_volume, zeroVolume = startMeasurment(calibration_json, pipe, measurment)
                            writer.writerow([measurment, round(measured_volume, 6), round(measured_volume - zeroVolume, 6), datetime.now()])
                    
                    print(f'Measurments finished. {no_iterrations} measurments saved in {save_file}. It took {(round(time.time()*1000) - milis_since_epoch)/1000} real time seconds to finish collecting data.')
                else:
                    print('Please also provide the name of the configuration and result save file without the ".json" or ".csv" sufix. After that you can also define the number of iterrations')
            
            elif(mode == 'optimise2'):
                decimal_factor = 0.1
                low = 0
                high = 20

                for decimal_place in range(3):
                    milis_since_epoch = round(time.time()*1000)
                    data = []
                    
                    # Create an Excel and add formating.
                    book = xlsxwriter.Workbook('xls_results/calibrate_scale_' + str(decimal_place) + '_' + str(milis_since_epoch) + '.xlsx')
                    bold = book.add_format({'bold': 1})
                    sheet = book.add_worksheet('distance errors')
                    
                    # define first line
                    fields = ['scale factor', 'average distance x-axis', 'Distance error x-axis', 'average distance y-axis', 'Distance error y-axis']
                    for n, field in enumerate(fields):
                        sheet.write(0, n, field, bold)

                    iterr = [round(decimal_factor + x * decimal_factor, 5) for x in range(int(low), int(high))]
                    data = findOptimalParameters(pipe, iterr)
                        
                    # iterating through the content list
                    for row, fields in enumerate(data):
                        sheet.write_number(row+1, 0, fields[0])
                        sheet.write_number(row+1, 1, fields[1])
                        sheet.write_number(row+1, 2, fields[2])
                        sheet.write_number(row+1, 3, fields[3])
                        sheet.write_number(row+1, 4, fields[4])

                    book.close()

                    # Find minimums and adjust the next step
                    min1 = min(list(map(lambda a: a[2], data)))
                    min2 = min(list(map(lambda a: a[4], data)))

                    ind1 = 0
                    ind2 = 0
                    for i in data:
                        if min1 == i[2]:
                            ind1 = i[0]
                        if min2 == i[4]:
                            ind2 = i[0]

                    low = round(ind1/decimal_factor - 1, 2)*10 if ind1 < ind2 else round(ind2/decimal_factor - 1, 2)*10
                    low = low if low > 0 else 0
                    high = round(ind1/decimal_factor + 1, 2)*10 if ind1 > ind2 else round(ind2/decimal_factor + 1, 2)*10
                    decimal_factor = round(decimal_factor * 0.1, 6)

                    print(f'\nLow: {low}  High: {high}  Step: {decimal_factor}\n')
                    print(f'For {decimal_place} decimal place found minimum at index: {ind1} with value of: {min1} and at index: {ind2} with value of: {min2}')


            elif(mode == 'optimise'):
                milis_since_epoch = round(time.time()*1000)
                list_name = ''
                for file in os.listdir("/assets"):
                    data = []
                    files_results = {
                        'errors': [],
                        'volumes': []
                    }

                    # Load data from files
                    pcd_o = loadPCD(file)
                    calibration_json = loadCalibration(file.split('.')[0])
                    
                    rotation_matrix = np.array(calibration_json['rotation'])
                    cropArea = np.array(calibration_json['crop area'])
                    lift_pcd = calibration_json['liftPcd']
                    cropArea_o = copy.deepcopy(cropArea)

                    # Set savefile
                    list_name = file.split('.')[:-2]
                    
                    # Create an Excel and add formating.
                    book = xlsxwriter.Workbook('xls_results/calibrate_volume_avrg_over_test_' + milis_since_epoch + '.xlsx')
                    bold = book.add_format({'bold': 1})

                    if list_name != file.split('.')[0][:-2]:
                        list_name = file.split('.')[0][:-2]
                        sheet = book.add_worksheet(list_name)
                        print(f'Starting measurment for {list_name}:\n')
                    
                    # define first line
                    fields = ["Scale factor", "Measured volume [m^3]", "Volume error [m^3]"]
                    for i, field in enumerate(fields):
                        sheet.write(0, i, field, bold)

                    iterr = [round(0.700 + x * 0.001, 4) for x in range(0, 301)] # from 0.7 to 0.73 by 0.0001
                    [measured_volumes, volume_errors] = optimiseWithAutofloorSize(pcd_o, rotation_matrix, cropArea_o, iterr)

                    # plotGeometriesWithOriginVectors([pcd_c, pcd_b])

                    files_results['volumes'].append(measured_volumes)
                    files_results['errors'].append(volume_errors)

                for n in range(len(iterr)):
                    volumes = []
                    errors = []
                    
                    for m in range(len(files_results['volumes'])):
                        volumes.append(files_results['volumes'][m][n])
                        errors.append(files_results['errors'][m][n])
                    
                    avrg_volume = average(volumes)
                    avrg_error = average(errors)

                    data.append([iterr[n], avrg_volume, avrg_error])
                    
                # iterating through the content list
                for row, fields in enumerate(data):
                    sheet.write_number(row+1, 0, fields[0])
                    sheet.write_number(row+1, 1, fields[1])
                    sheet.write_number(row+1, 2, fields[2])

                book.close()
                print(f'All measurments finished. It took {(round(time.time()*1000) - milis_since_epoch)/1000} seconds')


            elif(mode == 'measure_xls'):
                # get the wanted number of measurments
                file_name = sys.argv[2]
                no_iterrations = int(sys.argv[3]) if no_arguments >= 4 else 10
                list_names = sys.argv[4] if no_arguments >=5 else ''

                # Start  with measurments or configuration
                if(os.path.isfile('./configuration_files/' + file_name + '.json')):
                    # Load point coordinates and pcd rotation matrix
                    calibration_json = loadCalibration(file_name)
                else:
                    # If no file is present calibrate and load
                    print('Calibration file not found. Calibrating...')
                    captureReference(file_name, pipe, -1, True, True)

                    # Load point coordinates and pcd rotation matrix
                    calibration_json = loadCalibration(file_name)

                milis_since_epoch = round(time.time()*1000)
                save_file = file_name + '_' + str(milis_since_epoch)

                book = xlsxwriter.Workbook('xls_results/' + save_file + '.xlsx')
                
                # Add an Excel formating.
                bold = book.add_format({'bold': 1})
                time_format = book.add_format({'num_format': 'hh:mm:ss'})
                
                for list_name in list_names.split(','):
                    input(f'Press enter to continue measurment for {list_name}:\n')
                    sheet = book.add_worksheet(list_name)

                    # define first line
                    fields = ["Iterration number", "Volume [m^3]", "Volume difference [m^3]", "Time"]
                    for i, field in enumerate(fields):
                        sheet.write(0, i, field, bold)
                    
                    # get measurments with a pause between each picture
                    data = []       # ["Iterration number", "Volume [m^3]", "Volume difference [m^3]", "Time"]
                    for measurment in range(no_iterrations):
                        measured_volume, zeroVolume = startMeasurment(calibration_json, pipe, measurment)
                        data.append([measurment, measured_volume, measured_volume-zeroVolume, datetime.now()])

                    # iterating through the content list
                    for row, fields in enumerate(data):
                        sheet.write_number(row+1, 0, fields[0])
                        sheet.write_number(row+1, 1, fields[1])
                        sheet.write_number(row+1, 2, fields[2])
                        sheet.write_datetime(row+1, 3, fields[3], time_format)

                    # Define excel funciotns if necesarry
                    last_row = str(len(data)+1)
                    sheet.write_string(1, 5, 'Average [m^3]:')
                    sheet.write_formula(1, 6, '=AVERAGE(B2:B' + last_row + ')')
                    sheet.write_formula(1, 7, '=AVERAGE(C2:C' + last_row + ')')
                    
                    sheet.write_string(2, 5, 'Average (L):')
                    sheet.write_formula(2, 6, '=G2*1000')
                    sheet.write_formula(2, 7, '=H2*1000')

                    sheet.write_string(4, 5, 'Volume offset:')
                    sheet.write_formula(4, 6, '=B2-C2')

                    print(f'Measurment "{list_name}" finished. {no_iterrations} measurments saved in "{save_file}", sheet "{list_name}". It took {(round(time.time()*1000) - milis_since_epoch)/1000} real time seconds to finish collecting data.')
                
                # Close excel and finish
                book.close()
                print('All measurments finished.')
        else:
            print(f'The given argument is invalid, please enter only one of the following: {acceptableArgList}')
    
        # Stop streaming
        pipe.stop()
    else:
        print('Please enter the name of the function that you want to run')
        print('Syntax: .../main.py calibrate my_file_name  **or**  .../main.py calibrate my_file_name False False')
        print('Syntax: .../main.py measure my_save_and_config_name  **or**  .../main.py measure my_save_and_config_name 50 my_save_file_extencion')
        print('\nNote: Only the action and file name are necesary')

    

# This runs only in debbug mode, for developer use only
gettrace = getattr(sys, 'gettrace', None)
if gettrace is None:
    pass
elif gettrace():
    print('Hmm, Big Debugger is watching me')

    pipe = initCamera(True)
    captureReference('code_check', pipe, -1, True, True, 10, False)
    pipe.stop()