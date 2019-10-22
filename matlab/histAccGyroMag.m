function [meas] = histAccGyroMag()
% Collects data and creates histograms

  %% Setup necessary infrastructure
  import('se.hendeby.sensordata.*');  % Used to receive data.

  
  % Saved data.
  nx = 3;
 
  meas = struct('t', zeros(1, 0),...
                'acc', zeros(3, 0),...
                'gyr', zeros(3, 0),...
                'mag', zeros(3, 0),...
                'orient', zeros(4, 0));
  try
    %% Create data link
    server = StreamSensorDataReader(3400);
    % Makes sure to resources are returned.
    sentinel = onCleanup(@() server.stop());

    server.start();  % Start data reception.
  catch e
    fprintf(['Unsuccessful connecting to client!\n' ...
      'Make sure to start streaming from the phone *after* '...
             'running this function!']);
    return;
  end

  t0 = [];
  T = 30; % Seconds to store
  f = waitbar(0, 'Data collected');
  
  %% Filter loop
  while server.status()  % Repeat while data is available
      
    % Get the next measurement set, assume all measurements
    % within the next 5 ms are concurrent (suitable for sampling
    % in 100Hz).
    data = server.getNext(5);

    if isnan(data(1))  % No new data received
      continue;
    end
    t = data(1)/1000;  % Extract current time

    if isempty(t0)  % Initialize t0
      t0 = t;
    end

    if (t > t0 + T)
        break;
    end
    
    waitbar((t-t0)/T, f);
    
    gyr = data(1, 5:7)';
    acc = data(1, 2:4)';
    mag = data(1, 8:10)';
 
 
    meas.t(end+1) = t - t0;
    meas.acc(:, end+1) = acc;
    meas.gyr(:, end+1) = gyr;
    meas.mag(:, end+1) = mag;
  end
  close(f)
  
  figure(1)
  clf
  for i = 1:3
    subplot(1,3,i)
    hist(meas.acc(i,:))
  end
  title('Acc')
  
  figure(2)
  clf
  for i = 1:3
    subplot(1,3,i)
    hist(meas.gyr(i,:))
  end
  title('Gyro')
  
  figure(3)
  clf
  for i = 1:3
    subplot(1,3,i)
    hist(meas.mag(i,:))
  end
  title('Magnetometer')
  
end
