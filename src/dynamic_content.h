#ifndef DYNAMIC_CONTENT_H
#define DYNAMIC_CONTENT_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sdr_ui.h"

// Function to dynamically replace version string in HTML content
char* process_dynamic_content(const char* content, size_t content_length, size_t* new_length) {
    if (!content || content_length == 0) {
        *new_length = 0;
        return NULL;
    }

    // Look for version placeholder in the content
    const char* version_tag_start = "<h1>";
    const char* version_tag_end = "</h1>";
    char* pos_start = strstr(content, version_tag_start);
    
    // If we don't find the tag, return the original content
    if (!pos_start) {
        char* result = malloc(content_length + 1);
        if (!result) {
            *new_length = 0;
            return NULL;
        }
        memcpy(result, content, content_length);
        result[content_length] = '\0';
        *new_length = content_length;
        return result;
    }
    
    pos_start += strlen(version_tag_start);
    char* pos_end = strstr(pos_start, version_tag_end);
    
    if (!pos_end) {
        char* result = malloc(content_length + 1);
        if (!result) {
            *new_length = 0;
            return NULL;
        }
        memcpy(result, content, content_length);
        result[content_length] = '\0';
        *new_length = content_length;
        return result;
    }
    
    // Calculate sizes
    size_t prefix_len = pos_start - content;
    size_t suffix_len = content_length - (pos_end - content);
    size_t version_len = strlen(VER_STR);
    
    // Allocate memory for the new content
    *new_length = prefix_len + version_len + suffix_len;
    char* result = malloc(*new_length + 1);
    if (!result) {
        *new_length = 0;
        return NULL;
    }
    
    // Copy the parts
    memcpy(result, content, prefix_len);
    memcpy(result + prefix_len, VER_STR, version_len);
    memcpy(result + prefix_len + version_len, pos_end, suffix_len);
    result[*new_length] = '\0';
    
    return result;
}

#endif /* DYNAMIC_CONTENT_H */
