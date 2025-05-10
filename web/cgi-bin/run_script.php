<?php
// Simple script to list and execute shell scripts

// Force content type to be JSON
header('Content-Type: application/json');

// Make sure nothing is output before our JSON
ob_start();

// Basic error handling
try {
    // Configuration
    $scripts_dir = '/home/pi/sbitx/web/scripts';
    
    // Debug information
    $debug = [
        'time' => date('Y-m-d H:i:s'),
        'script_dir_exists' => file_exists($scripts_dir) ? 'yes' : 'no',
        'script_dir_is_dir' => is_dir($scripts_dir) ? 'yes' : 'no',
        'script_dir_readable' => is_readable($scripts_dir) ? 'yes' : 'no'
    ];
    
    // Get list of scripts
    $scripts = [];
    if (is_dir($scripts_dir) && is_readable($scripts_dir)) {
        $files = scandir($scripts_dir);
        foreach ($files as $file) {
            if (substr($file, -3) === '.sh' && is_file("$scripts_dir/$file")) {
                $scripts[] = $file;
            }
        }
    }
    
    // Create response
    $response = [
        'status' => 'success',
        'scripts' => $scripts,
        'debug' => $debug
    ];
    
    // Handle requests based on method
    $method = isset($_SERVER['REQUEST_METHOD']) ? $_SERVER['REQUEST_METHOD'] : 'GET';
    
    if ($method === 'GET') {
        // Just return the list of scripts
        echo json_encode($response);
    } else if ($method === 'POST') {
        // Execute a script if requested
        $script_name = isset($_POST['script']) ? $_POST['script'] : '';
        
        if (!$script_name) {
            $response['status'] = 'error';
            $response['message'] = 'No script specified';
        } else if (!preg_match('/^[a-zA-Z0-9_-]+\.sh$/', $script_name)) {
            $response['status'] = 'error';
            $response['message'] = 'Invalid script name';
        } else if (!in_array($script_name, $scripts)) {
            $response['status'] = 'error';
            $response['message'] = "Script $script_name not found";
        } else {
            // Execute the script
            $script_path = "$scripts_dir/$script_name";
            $command = "nohup sudo /bin/bash " . escapeshellarg($script_path) . " > /dev/null 2>&1 &";
            exec($command);
            $response['message'] = "Script $script_name started successfully";
        }
        
        echo json_encode($response);
    } else {
        // Method not allowed
        http_response_code(405);
        echo json_encode(['status' => 'error', 'message' => 'Method not allowed']);
    }
} catch (Exception $e) {
    // Handle any exceptions
    http_response_code(500);
    echo json_encode([
        'status' => 'error',
        'message' => $e->getMessage()
    ]);
}

// Clean any buffered output and send only our JSON
ob_end_flush();
// Make sure nothing is added after this point
exit;