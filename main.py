from basic import loadCalibration, startMeasurment
from cameraF import initCamera
from calibrate import captureReference
from datetime import datetime
import sys, csv, time, os, xlsxwriter


if __name__ == '__main__':   
    no_arguments = len(sys.argv)
    acceptableArgList = ['calibrate', 'measure', 'help', 'measure_xls']

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
                    data = []
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

                    sheet.write_string(4, 5, 'Max [m^3]:')
                    sheet.write_formula(4, 7, '=MAX(C2:C' + last_row + ')')

                    sheet.write_string(5, 5, 'Min [m^3]:')
                    sheet.write_formula(5, 7, '=MIN(C2:C' + last_row + ')')

                    sheet.write_string(6, 5, 'Standard deviation [m^3]:')
                    sheet.write_formula(6, 7, '=STDEV(C2:C' + last_row + ')')

                    sheet.write_string(8, 5, 'Volume offset:')
                    sheet.write_formula(8, 6, '=B2-C2')

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