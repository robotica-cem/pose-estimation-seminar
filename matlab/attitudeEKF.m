function [xhat, meas] = filterTemplate(calAcc, calGyr, calMag)
% FILTERTEMPLATE  Filter template
%
% This is a template function for how to collect and filter data
% sent from a smartphone live.  Calibration data for the
% accelerometer, gyroscope and magnetometer assumed available as
% structs with fields m (mean) and R (variance).
%
% The function returns xhat as an array of structs comprising t
% (timestamp), x (state), and P (state covariance) for each
% timestamp, and meas an array of structs comprising t (timestamp),
% acc (accelerometer measurements), gyr (gyroscope measurements),
% mag (magnetometer measurements), and orint (orientation quaternions
% from the phone).  Measurements not availabe are marked with NaNs.
%
% As you implement your own orientation estimate, it will be
% visualized in a simple illustration.  If the orientation estimate
% is checked in the Sensor Fusion app, it will be displayed in a
% separate view.

  %% Setup necessary infrastructure
  import('se.hendeby.sensordata.*');  % Used to receive data.

  %% Orientation viewer settings
  % Note that the global coordinate system is East(x)-North(y)-Up(z)
  % The view is towards the origin of the global coordinate system
  % and from the cameraposition you set
  camerapos = [0,-5,0]; % This is the default. Looking from straight south
  camerapos = [5,0,0]; % This is looking from the east towards the setting sun
  camerapos = [4,-3,0]; % This is looking from south-east

  refreshrate = 20; % Approximate refreshrate
  
  %% Filter settings
  measurement_update = @mu_acc; % Function handle
  measurement_update = @mu_acc_robust; 
  measurement_update = @mu_acc_outlier_reject; 
  
  tolAcc = 2;
  tolMag = tolAcc;
  
  t0 = [];  % Initial time (initialize on first data received)
  nx = 4;
  % Add your filter settings here.
  Q = 1e-2*eye(4);
  Ra = 1e-1*eye(3);
  Rm = eye(3); % Without normalization of the magnetic field vector
  Rm = 1e-2*eye(3); % With normalization
  
  g0 = [0;0;9.8527];
  m0 = [0; 24.53; -21.24];
  %tolAcc = 4*norm(g0);
  %tolMag = 0.4*norm(m0);
  normalizeAcc = 0;
  normalizeMag = 1;
  
  
  % Current filter state.
  x = [1; 0; 0 ;0];
  P = eye(nx, nx);

  % Saved filter states.
  xhat = struct('t', zeros(1, 0),...
                'x', zeros(nx, 0),...
                'P', zeros(nx, nx, 0));

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

  % Used for visualization.
  figure(1);
  subplot(1, 2, 1);
  ownView = OrientationView('Own filter', gca);  % Used for visualization.
  gvecl = plot3 ([0 0], [0,0], [0,0], 'm', 'linewidth', 2);
  mvecl = plot3 ([0 0], [0,0], [0,0], 'c', 'linewidth', 2);
  title(ownView, 'OWN', 'FontSize', 16);
  set(gca, 'CameraPosition', camerapos);
  googleView = [];
  lastrefresh = [];
 
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
      tlast = t0;
    end
    if isempty(lastrefresh)  % Initialize t0
      lastrefresh = t;
    end

    h = t-tlast;
    gyr = data(1, 5:7)';
    if ~any(isnan(gyr))  % Gyro measurements are available.
       h = t-tlast;
        if h>0
            [x,P] = tu_gyr(x,P,gyr,Q,h);
            tlast = t;
        end
            
            % Do something
    end

    acc = data(1, 2:4)';
    if ~any(isnan(acc))  % Acc measurements are available.
        [x,P] = measurement_update(x, P, acc, Ra, g0, tolAcc, normalizeAcc);
        
        % Do something
    end

    mag = data(1, 8:10)';
    if ~any(isnan(mag))  % Mag measurements are available.
      % Do something
    [x,P] = measurement_update(x, P, mag, Rm, m0, tolMag, normalizeMag );
        
    end

    orientation = data(1, 18:21)';  % Google's orientation estimate.

    R = Qq(x(1:4));
    gg = R*acc/9.8;
    mm = R*mag/norm(m0);
    % Visualize result
    if (t - lastrefresh) > 1.0/refreshrate
      setOrientation(ownView, x(1:4));
      set(gvecl, 'XData', [0 gg(1)], 'YData', [0 gg(2)], 'ZData', [0 gg(3)]);
      set(mvecl, 'XData', [0 mm(1)], 'YData', [0 mm(2)], 'ZData', [0 mm(3)]);
      if ~any(isnan(orientation))
        if isempty(googleView)
          subplot(1, 2, 2);
          % Used for visualization.
          googleView = OrientationView('Google filter', gca);
          set(gca, 'CameraPosition', camerapos);
        end
        setOrientation(googleView, orientation);
        title(googleView, 'GOOGLE', 'FontSize', 16);
      end
      drawnow
      lastrefresh = t;
    end
    

    % Save estimates
    xhat.x(:, end+1) = x;
    xhat.P(:, :, end+1) = P;
    xhat.t(end+1) = t - t0;

    meas.t(end+1) = t - t0;
    meas.acc(:, end+1) = acc;
    meas.gyr(:, end+1) = gyr;
    meas.mag(:, end+1) = mag;
    meas.orient(:, end+1) = orientation;
  end
end
