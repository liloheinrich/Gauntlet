% map the gauntlet by getting lidar data from neato

% clear all;
% rostopic list
scan('data_files/scandata')

function scan(datasetname)
% Method for collecting lidar scan data from Neato. 
% - To launch, call this method. 
% - To start execution of the program, press space bar. 
% - To stop execution of the program, close the figure window. 
% - Collected data (ranges, ange_increment) is stored under 'datasetname.mat'

    function myCloseRequest(src,callbackdata)
        % Close request function to display a question dialog box
        % get rid of subscriptions to avoid race conditions
        clear ranges;
        clear angle_increment;
        delete(gcf)
    end

    function processScan(sub, msg)
        ranges = msg.Ranges;
        angle_increment = msg.AngleIncrement;
    end

    function keyPressedFunction(fig_obj, eventDat)
        % Convert a key pressed event into a twist message and publish it
        ck = get(fig_obj, 'CurrentKey');
        switch ck
            case 'space'
                if collectingData
                    collectingData = false;
                    save(datasetname, 'ranges', 'angle_increment');
                    disp('Stopping dataset collection');
                else
                    collectingData = true;
                    disp('Starting dataset collection');
                end
        end
    end

    global ranges angle_increment;
    collectingData = false;
    ranges = zeros(1,1);
    angle_increment = 0;
    sub_scan = rossubscriber('/scan', @processScan);

	f = figure('CloseRequestFcn',@myCloseRequest);
    title('Dataset Collection Window');
    set(f,'WindowKeyPressFcn', @keyPressedFunction);
end