<?php
// Simple script to list and execute start/stop shell scripts for ham radio applications

// Force content type to be JSON
header('Content-Type: application/json');

// Make sure nothing is output before our JSON
ob_start();

// Basic error handling
try {
    // Configuration
    $scripts_dir = '/home/pi/sbitx/web/scripts';
    $valid_apps = ['wsjtx', 'fldigi', 'js8call']; // Valid application names

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
            if (preg_match('/^(start|stop)_[a-zA-Z0-9_-]+\.sh$/', $file) && is_file("$scripts_dir/$file")) {
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
        // Return the list of scripts
        echo json_encode($response);
    } else if ($method === 'POST') {
        // Execute a start or stop script
        $app_name = isset($_POST['app']) ? $_POST['app'] : '';
        $action = isset($_POST['action']) ? $_POST['action'] : '';

        if (!$app_name || !$action) {
            $response['status'] = 'error';
            $response['message'] = 'Missing app or action parameter';
        } else if (!in_array($app_name, $valid_apps)) {
            $response['status'] = 'error';
            $response['message'] = 'Invalid application name';
        } else if (!in_array($action, ['start', 'stop'])) {
            $response['status'] = 'error';
            $response['message'] = 'Invalid action';
        } else {
            $script_name = "{$action}_{$app_name}.sh";
            $script_path = "$scripts_dir/$script_name";

            if (!in_array($script_name, $scripts)) {
                $response['status'] = 'error';
                $response['message'] = "Script $script_name not found";
            } else {
                // Execute the script as pi user
                $command = "/bin/bash " . escapeshellarg($script_path) . " 2>&1";
                exec($command, $output, $return_var);
                $response['message'] = implode("\n", $output);
                $response['return_code'] = $return_var;
                if ($return_var !== 0) {
                    $response['status'] = 'error';
                    $response['message'] = "Failed to execute $script_name: " . $response['message'];
                }
            }
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
