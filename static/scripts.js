function startVideo() {
    var timestamp = new Date().getTime();
    document.getElementById('video-stream').src = "/get_video_frames" + "?timestamp=" + timestamp;
}

function startVideo2() {
    var timestamp = new Date().getTime();
    document.getElementById('video-stream').src = "/get_placeholder_frames" + "?timestamp=" + timestamp;
}

function updateShowVideo(prev) {
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/get_show_video", true);
    xhr.onreadystatechange = function() {
        if (xhr.readyState == 4 && xhr.status == 200) {
            var showVideoValue = JSON.parse(xhr.responseText).show_video;
            if (showVideoValue !== prev) {
                if (showVideoValue) {
                    startVideo();
                } else {
                    startVideo2();
                }
                console.log(showVideoValue);
                console.log(prev);
                prev_show_video = showVideoValue;
            }
        }
    };
    xhr.send();
}