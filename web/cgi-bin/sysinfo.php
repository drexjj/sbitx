<?php
	$start_time = microtime(TRUE);

	$operating_system = PHP_OS_FAMILY;

	if ($operating_system === 'Windows') {
		// Win CPU
		$wmi = new COM('WinMgmts:\\\\.');
		$cpus = $wmi->InstancesOf('Win32_Processor');
		$cpuload = 0;
		$cpu_count = 0;
		foreach ($cpus as $key => $cpu) {
			$cpuload += $cpu->LoadPercentage;
			$cpu_count++;
		}
		// WIN MEM
		$res = $wmi->ExecQuery('SELECT FreePhysicalMemory,FreeVirtualMemory,TotalSwapSpaceSize,TotalVirtualMemorySize,TotalVisibleMemorySize FROM Win32_OperatingSystem');
		$mem = $res->ItemIndex(0);
		$memtotal = round($mem->TotalVisibleMemorySize / 1000000,2);
		$memavailable = round($mem->FreePhysicalMemory / 1000000,2);
		$memused = round($memtotal-$memavailable,2);
		// WIN CONNECTIONS
		$connections = shell_exec('netstat -nt | findstr :' . $_SERVER['SERVER_PORT'] . ' | findstr ESTABLISHED | find /C /V ""');
		$totalconnections = shell_exec('netstat -nt | findstr :' . $_SERVER['SERVER_PORT'] . ' | find /C /V ""');
	} else {
		// Linux CPU
		$load = sys_getloadavg();
		$cpuload = round($load[0], 2);
#		$cpuload = $load[0];
		$cpu_count = shell_exec('nproc');
		// Linux MEM
		$free = shell_exec('free');
		$free = (string)trim($free);
		$free_arr = explode("\n", $free);
		$mem = explode(" ", $free_arr[1]);
		$mem = array_filter($mem, function($value) { return ($value !== null && $value !== false && $value !== ''); }); // removes nulls from array
		$mem = array_merge($mem); // puts arrays back to [0],[1],[2] after 
		$memtotal = round($mem[1] / 1000000,3);
		$memused = round($mem[2] / 1000000,3);
		$memfree = round($mem[3] / 1000000,3);
		$memshared = round($mem[4] / 1000000,2);
		$memcached = round($mem[5] / 1000000,2);
		$memavailable = round($mem[6] / 1000000,2);
		// Linux Connections
		$connections = `netstat -ntu | grep -E ':80 |443 ' | grep ESTABLISHED | grep -v LISTEN | awk '{print $5}' | cut -d: -f1 | sort | uniq -c | sort -rn | grep -v 127.0.0.1 | wc -l`; 
		$totalconnections = `netstat -ntu | grep -E ':80 |443 ' | grep -v LISTEN | awk '{print $5}' | cut -d: -f1 | sort | uniq -c | sort -rn | grep -v 127.0.0.1 | wc -l`; 
		$model = shell_exec('cat /proc/cpuinfo | grep Model | cut -d: -f 2');
		$cputemp = shell_exec('sudo vcgencmd measure_temp | cut -d= -f 2');
    $wifi = shell_exec("if [ \$(iwconfig wlan0 | grep -oP 'Mode:\K\w+') = 'Managed' ]; then 
                       iwconfig wlan0 | grep -oP 'ESSID:\".*\"'; 
                   else 
                       iw dev wlan0 info | grep ssid | awk '{print \"ESSID:\\\"\" \$2 \"\\\"\"}'; 
                   fi && 
                   ip addr show wlan0 | grep -oP 'inet \K[\d.]+' | paste -d ' ' - -");
		$time = shell_exec('date');
	}

	$memusage = round(($memused/$memtotal)*100);		

	$phpload = round(memory_get_usage() / 1000000,2);

	$diskfree = round(disk_free_space(".") / 1000000000,3);
	$disktotal = round(disk_total_space(".") / 1000000000,3);
	$diskused = round($disktotal - $diskfree,3);
	$diskusage = round($diskused/$disktotal*100);

?>


<?php
// Return just the data without HTML structure for AJAX requests
// No need for header.php inclusion
?>

<div class="system-data">
		<p><br><nobr><strong><?php echo $model; ?></strong></nobr></p>
<hr width="400px" align="left">


<p>GPS </p>
<ul>
<?php
echo "<pre style=\"font-family: variable;\">";
$gps_devices_found = false;
$gps_device_path = '';

if (file_exists('/dev/serial/by-id')) {
   $handle = opendir('/dev/serial/by-id');
   while (false !== ($entry = readdir($handle))) {
       if ($entry != "." && $entry != ".." ) {
         $link = readlink("/dev/serial/by-id/" . $entry);
         $link = substr($link, 6);
         echo($link . ":  ");
         echo $entry . "<br>";
         $gps_devices_found = true;
         
         // Store the device path for potential direct access
         if (strpos(strtolower($entry), 'gps') !== false || 
             strpos(strtolower($entry), 'gnss') !== false || 
             strpos(strtolower($entry), 'u-blox') !== false) {
             $gps_device_path = '/dev/' . $link;
         }
       }
   }
   closedir($handle);
}
// Also check for ttyACM and ttyUSB devices if no GPS was found by ID
if (!$gps_devices_found) {
    foreach (glob('/dev/ttyACM*') as $device) {
        echo basename($device) . "<br>";
        $gps_devices_found = true;
        $gps_device_path = $device;
    }
    
    if (!$gps_devices_found) {
        foreach (glob('/dev/ttyUSB*') as $device) {
            echo basename($device) . "<br>";
            $gps_devices_found = true;
            $gps_device_path = $device;
        }
    }
    
    if (!$gps_devices_found) {
        echo "--- no serial devices ---";
    }
}

echo "</pre>";

// Check if gpsd is running
$gpsd_running = trim(shell_exec("pgrep -x gpsd > /dev/null && echo 'yes' || echo 'no'"));

// Function to generate simulated satellite data for testing
function generate_simulated_satellites() {
    $satellites = [];
    $num_satellites = rand(8, 16);
    $num_used = rand(4, $num_satellites);
    
    for ($i = 1; $i <= $num_satellites; $i++) {
        $is_used = $i <= $num_used;
        $satellites[] = [
            'prn' => $i,
            'ss' => rand(10, 45), // Signal strength between 10-45 dB
            'el' => rand(5, 90),  // Elevation between 5-90 degrees
            'az' => rand(0, 359), // Azimuth between 0-359 degrees
            'used' => $is_used
        ];
    }
    
    return [
        'satellites' => $satellites,
        'sat_count' => $num_satellites,
        'satellites_in_use' => $num_used
    ];
}

// Initialize arrays for satellite data
$satellites = [];
$sat_count = 0;
$satellites_in_use = 0;
$data_source = 'none';

// Try to get real satellite data if GPS device is found
if ($gps_devices_found) {
    // Method 1: Try gpsd if it's running
    if ($gpsd_running == 'yes') {
        // Try to connect to gpsd
        $socket = @fsockopen('localhost', 2947, $errno, $errstr, 1);
        
        if ($socket) {
            // Send WATCH command to enable JSON output
            fwrite($socket, "?WATCH={\"enable\":true,\"json\":true};\n");
            // Wait a moment for gpsd to process
            usleep(100000); // 100ms
            // Send POLL command to get current data
            fwrite($socket, "?POLL;\n");
            
            // Read response
            $gps_data = '';
            $timeout = time() + 2; // 2 second timeout
            
            while (!feof($socket) && time() < $timeout) {
                $gps_data .= fread($socket, 4096);
                // If we've received enough data, break
                if (strpos($gps_data, '"class":"SKY"') !== false) {
                    break;
                }
                usleep(50000); // 50ms pause between reads
            }
            
            fclose($socket);
            
            // Parse JSON data
            if (!empty($gps_data)) {
                $lines = explode("\n", $gps_data);
                foreach ($lines as $line) {
                    if (empty(trim($line))) continue;
                    
                    // Try to decode JSON, but handle malformed responses
                    try {
                        $json_data = @json_decode($line, true);
                        
                        if (json_last_error() == JSON_ERROR_NONE && 
                            isset($json_data['class']) && 
                            $json_data['class'] == 'SKY' && 
                            isset($json_data['satellites']) &&
                            count($json_data['satellites']) > 0) {
                            
                            $sat_count = count($json_data['satellites']);
                            
                            foreach ($json_data['satellites'] as $sat) {
                                if (isset($sat['PRN'])) {
                                    $satellites[] = [
                                        'prn' => $sat['PRN'],
                                        'ss' => isset($sat['ss']) ? $sat['ss'] : 0,
                                        'el' => isset($sat['el']) ? $sat['el'] : 0,
                                        'az' => isset($sat['az']) ? $sat['az'] : 0,
                                        'used' => isset($sat['used']) && $sat['used'] ? true : false
                                    ];
                                    
                                    if (isset($sat['used']) && $sat['used']) {
                                        $satellites_in_use++;
                                    }
                                }
                            }
                            
                            $data_source = 'gpsd';
                            break;
                        }
                    } catch (Exception $e) {
                        // Skip malformed JSON
                        continue;
                    }
                }
            }
        }
    }
    
    // Method 2: Try direct device access if gpsd didn't provide data
    if (empty($satellites) && !empty($gps_device_path) && file_exists($gps_device_path)) {
        // This would require parsing NMEA sentences directly
        // For simplicity in this implementation, we'll skip this method
        // and use simulation instead
    }
}


    
    // Display satellite bar chart if satellites were found
    if (count($satellites) > 0) {
        echo "<div class='gps-stats'>";
        
        // Show satellite counts only (remove simulation/live label)
        echo "<div class='gps-header'>";
        echo "<p>Satellites: <strong>{$sat_count}</strong> visible, <strong>{$satellites_in_use}</strong> in use</p>";
        echo "</div>";
        
        // Signal strength legend
        echo "<div class='signal-legend'>";
        echo "<div class='legend-item'><span class='legend-color' style='background-color:#4CAF50;'></span>In Use</div>";
        echo "<div class='legend-item'><span class='legend-color' style='background-color:#607D8B;'></span>Visible</div>";
        echo "</div>";
        
        echo "<div class='sat-chart'>";
        
        // Sort satellites by signal strength
        usort($satellites, function($a, $b) {
            return $b['ss'] - $a['ss'];
        });
        
        // Display grid lines for SNR (10,20,30,40,50)
        $chart_height = 60; // px, must match .snr-grid height
        echo "<div class='snr-grid'>";
        for ($snr = 50; $snr >= 10; $snr -= 10) {
            $top = ($chart_height - ($snr / 50) * $chart_height);
            echo "<div class='snr-grid-line' style='top: {$top}px;'><span class='snr-label'>{$snr}</span></div>";
        }
        echo "</div>";
        // Display bar chart
        foreach ($satellites as $sat) {
            $signal_px = max(5, intval(($sat['ss'] / 50) * 60)); // 0-60px, min 5px
            $bar_color = $sat['used'] ? '#4CAF50' : '#FFD600'; // Green if used, yellow if not
            echo "<div class='sat-bar-container'>";
            echo "<div class='signal-above-bar'>{$sat['ss']}</div>";
            echo "<div class='sat-bar' style='height:{$signal_px}px; background-color:{$bar_color};' title='PRN: {$sat['prn']}\nSignal: {$sat['ss']} dB'></div>";
            echo "<div class='sat-label'>{$sat['prn']}</div>";
            echo "</div>";
        }
        
        echo "</div>";
        
        // Add CSS for the satellite chart
        echo "<style>
            .gps-stats {
                margin-top: 5px;
                margin-bottom: 5px;
                position: relative;
            }
            .gps-header {
                display: flex;
                justify-content: space-between;
                align-items: center;
                margin-bottom: 3px;
            }
            .data-source {
                font-size: 9px;
                padding: 1px 4px;
                border-radius: 3px;
                color: white;
                font-weight: bold;
            }
            .signal-legend {
                display: flex;
                align-items: center;
                margin-right: 6px;
                font-size: 9px;
            }
            .legend-color {
                width: 10px;
                height: 10px;
                margin-right: 3px;
                border-radius: 2px;
            }
            .sat-chart {
                display: flex;
                align-items: stretch;
                height: 60px;
                background-color: #1a1e2a;
                border-radius: 5px;
                padding: 3px 3px 0 3px;
                margin-top: 3px;
                overflow-x: auto;
                position: relative;
                border: 1px solid #333;
            }
            .sat-bar-container {
                display: flex;
                flex-direction: column;
                align-items: center;
                margin: 0 2px;
                min-width: 16px;
            }
            .sat-bar {
                width: 12px;
                border-radius: 2px 2px 0 0;
                transition: height 0.3s ease;
                margin-bottom: 0;
            }
            .signal-above-bar {
                font-size: 9px;
                color: #fff;
                margin-bottom: 2px;
                line-height: 1;
                text-shadow: 0 0 2px #000;
            }
            .sat-label {
                font-size: 9px;
                margin-top: 2px;
                color: #ccc;
            }
            .snr-grid {
                position: absolute;
                left: -22px;
                width: calc(100% + 22px);
                height: 100%;
                top: 0;
                pointer-events: none;
            }
            .snr-grid-line {
                position: absolute;
                left: 22px;
                width: calc(100% - 22px);
                border-top: 1px dashed #444;
                font-size: 8px;
                color: #888;
                height: 0;
            }
            .snr-label {
                position: absolute;
                left: -22px;
                top: -7px;
                width: 20px;
                text-align: right;
                background: #1a1e2a;
                padding: 0 2px 0 0;
                color: #888;
                font-size: 8px;
                line-height: 1;
            }
            .signal-above-bar {
                font-size: 9px;
                color: #fff;
                margin-bottom: 2px;
                line-height: 1;
                text-align: center;
                width: 100%;
                text-shadow: 0 0 2px #000;
            }
            .sat-bar-container {
                display: flex;
                flex-direction: column;
                align-items: center;
                justify-content: flex-end;
                margin: 0 2px;
                min-width: 16px;
                width: 18px;
                height: 100%;
            }
            .sat-bar {
                width: 12px;
                margin: 0 auto;
                border-radius: 2px 2px 0 0;
                transition: height 0.3s ease;
                margin-bottom: 0;
            }
            .sat-label {
                font-size: 9px;
                margin-top: 2px;
                color: #ccc;
                text-align: center;
                width: 100%;
                margin-bottom: 0;
            }
        </style>";
    } else if ($gps_devices_found) {
        echo "<p>No satellite data available</p>";
    }
 

if ($gps_devices_found && $gpsd_running != 'yes') {
    echo "<p>GPS device detected but gpsd is not running</p>";
}
?>
</ul>

<hr width="400px" align="left">
<p>Audio Interface</p>
<ul>
<?php
echo "<pre style=\"font-family: variable;\">";
$output = shell_exec("cat /proc/asound/cards | tr -s '  ' ");
echo $output . "</pre>";
?>
</ul>
<hr width="400px" align="left">
</p>
		<p>Memory Usage: <?php echo $memusage; ?>%</p>
		<p>CPU Load Average: <?php echo $cpuload; ?></p>
		<p>Disk Usage: <?php echo $diskusage; ?>%</p>
		<p>Network Connections: <?php echo $totalconnections; ?></p>
		<p>WiFi: <?php echo "<nobr>" . $wifi . "</nobr>"; ?></p>
		<p>CPU Count: <?php echo $cpu_count; ?></p>
		<p>CPU Temp: <?php echo $cputemp; ?></p>
		<p>Mem Total: <?php echo $memtotal; ?> GB</p>
		<p>Mem Used: <?php echo $memused; ?> GB</p>
		<p>Mem Available: <?php echo $memavailable; ?> GB</p>
		<p>SD Free: <?php echo $diskfree; ?> GB</p>
		<p>SD Used: <?php echo $diskused; ?> GB</p>
		<p>SD Total: <?php echo $disktotal; ?> GB</p>
		<p>Host Name: <?php echo $hostname = !empty($_SERVER['SERVER_NAME']) ? $_SERVER['SERVER_NAME'] : trim(shell_exec('hostname'));; ?></p>
		<p>Host Address: <?php echo $hostaddress = !empty($_SERVER['SERVER_ADDR']) ? $_SERVER['SERVER_ADDR'] : trim(shell_exec('hostname -I | awk \'{print $1}\''));; ?></p>
		<p>System Clock: <?php echo "<nobr>" . $time . "</nobr>"; ?></p>
<br>
</div>
