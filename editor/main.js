const {Tween, Easing, Group} = TWEEN;

let playback_buffer = new Array(), original_playback_buffer = new Array();

let undo_stack = new Array();
let redo_stack = new Array();

let tweens = new Array();

let stopNextTick = false;
let isRunning = false;
let frameCount = 0;
let playback_cursor = 0;
let previous_position = 0;
let fps = 0, fpsInterval = 0, startTime = 0, now = 0, then = 0, elapsed = 0, recordingLength = 0, originPosX = 0, originPosY = 0, originHeading = 0;

let coords = {x: -50, y: -50};
let alpha = {val: 0};

function bytes_to_double(byte_array) {
    return new Float64Array(byte_array)[0];
}
function bytes_to_float(data) {
    // Create a buffer
    var buf = new ArrayBuffer(4);
    // Create a data view of it
    var view = new DataView(buf);

    // set bytes
    data.forEach(function (b, i) {
        view.setUint8(data.length - i - 1, b);
    });

    // Read the bits as a float; note that by doing this, we're implicitly
    // converting it from a 32-bit float into JavaScript's native 64-bit double
    var num = view.getFloat32(0);
    // Done
    return num;
}
function float_to_byte_array(value) {
    let farr = new Float32Array(1);
    farr[0] = value;
    return new Int8Array(farr.buffer);
}

function duplicate_action() {
    let amount = Number(document.getElementById("duplication_captures").value);
    let recent_action = undo_stack[undo_stack.length - 1];
    if (recent_action.action != "update_buffer") return;
    for (let i = 1; i <= amount; i++) {
        playback_buffer[playback_cursor + i][recent_action.common_data[0]][recent_action.common_data[1]] = recent_action.new_data[0]
    };

    update_ui(playback_cursor);
};

function add_undo_action(action) {
    undo_stack.push(action);
    redo_stack.length = 0;
};

function undo() {
    if (undo_stack.length == 0) return;
    let undo_action = undo_stack.pop();

    switch (undo_action.action) {
        case "update_buffer":
            playback_cursor = undo_action.cursor_position;
            playback_buffer[playback_cursor][undo_action.common_data[0]][undo_action.common_data[1]] = undo_action.old_data[0];
            break;
        case "move_cursor":
            playback_cursor = undo_action.old_data[0];
            break;
        default:
            break;
    }

    redo_stack.push(undo_action);

    update_ui(playback_cursor);
};

function redo() {
    if (redo_stack.length == 0) return;
    let redo_action = redo_stack.pop();

    switch (redo_action.action) {
        case "update_buffer":
            playback_cursor = redo_action.cursor_position;
            playback_buffer[playback_cursor][redo_action.common_data[0]][redo_action.common_data[1]] = redo_action.new_data[0];
            break;
        case "move_cursor":
            playback_cursor = redo_action.new_data[0];
            break;
        default:
            break;
    };

    undo_stack.push(redo_action);

    update_ui(playback_cursor);
};

function resetBuffer() {
    playback_buffer = structuredClone(original_playback_buffer);
    playback_cursor = 0;
    update_ui(playback_cursor);
};

function save() {
    let serialized_buffer = new Int8Array(13 + playback_buffer.length * (16 + 12));
    let i = 0;
    serialized_buffer[i++] = recordingLength;
    let originBuffer = float_to_byte_array(originPosX);
    originBuffer.forEach(element => {
        serialized_buffer[i++] = element;
    });
    /////////////
    originBuffer = float_to_byte_array(originPosY);
    originBuffer.forEach(element => {
        serialized_buffer[i++] = element;
    });
    originBuffer = float_to_byte_array(originHeading);
    originBuffer.forEach(element => {
        serialized_buffer[i++] = element;
    });
    playback_buffer.forEach(controllerData => {
        serialized_buffer[i++] = controllerData.axis[0];
        serialized_buffer[i++] = controllerData.axis[1];
        serialized_buffer[i++] = controllerData.axis[2];
        serialized_buffer[i++] = controllerData.axis[3];
        serialized_buffer[i++] = controllerData.digital[0];
        serialized_buffer[i++] = controllerData.digital[1];
        serialized_buffer[i++] = controllerData.digital[2];
        serialized_buffer[i++] = controllerData.digital[3];
        serialized_buffer[i++] = controllerData.digital[4];
        serialized_buffer[i++] = controllerData.digital[5];
        serialized_buffer[i++] = controllerData.digital[6];
        serialized_buffer[i++] = controllerData.digital[7];
        serialized_buffer[i++] = controllerData.digital[8];
        serialized_buffer[i++] = controllerData.digital[9];
        serialized_buffer[i++] = controllerData.digital[10];
        serialized_buffer[i++] = controllerData.digital[11];
        let gpsDataBuffer = float_to_byte_array(controllerData.gps.x);
        gpsDataBuffer.forEach(element => {
            serialized_buffer[i++] = element;
        });
        gpsDataBuffer = float_to_byte_array(controllerData.gps.y);
        gpsDataBuffer.forEach(element => {
            serialized_buffer[i++] = element;
        });
        gpsDataBuffer = float_to_byte_array(controllerData.gps.heading);
        gpsDataBuffer.forEach(element => {
            serialized_buffer[i++] = element;
        });
    });

    let a = document.createElement("a");
    document.body.appendChild(a);
    a.style = "display: none";
    
    let blob = new Blob([serialized_buffer], {type: "application/octet-stream"});
    let url = window.URL.createObjectURL(blob);

    a.href = url;
    a.download = document.getElementById("input").files[0].name;
    a.click();

    window.URL.revokeObjectURL(url);

    a.remove();
};

function add_cursor(idx) {
    add_undo_action({
        "action": "move_cursor",
        "common_data": [],
        "old_data": [Number(playback_cursor)],
        "new_data": [Number(playback_cursor) + Number(idx)],
        "cursor_position": playback_cursor
    });
    
    playback_cursor += idx;
    update_ui(playback_cursor);
};

function stop_playback() {
    if (isRunning) stopNextTick = true;
    else update_ui(0);
};

function update_ui(idx) {
    $("#progress").val(idx.toString());
    $("#timeline").val(idx.toString());
    $("#timestamp").text(String(idx * (5 / 1000)).substring(0, 5) + " / " + (playback_buffer.length * (5/1000)) + "s");

    $("#axis1").val(playback_buffer[idx].axis[0]);
    $("#axis2").val(playback_buffer[idx].axis[1]);
    $("#axis3").val(playback_buffer[idx].axis[2]);
    $("#axis4").val(playback_buffer[idx].axis[3]);

    $("#button_a")    .prop("checked", playback_buffer[idx].digital[0] == 1);
    $("#button_b")    .prop("checked", playback_buffer[idx].digital[1] == 1);
    $("#button_x")    .prop("checked", playback_buffer[idx].digital[2] == 1);
    $("#button_y")    .prop("checked", playback_buffer[idx].digital[3] == 1);
    $("#button_up")   .prop("checked", playback_buffer[idx].digital[4] == 1);
    $("#button_right").prop("checked", playback_buffer[idx].digital[5] == 1);
    $("#button_down") .prop("checked", playback_buffer[idx].digital[6] == 1);
    $("#button_left") .prop("checked", playback_buffer[idx].digital[7] == 1);
    $("#button_l1")   .prop("checked", playback_buffer[idx].digital[8] == 1);
    $("#button_l2")   .prop("checked", playback_buffer[idx].digital[9] == 1);
    $("#button_r1")   .prop("checked", playback_buffer[idx].digital[10] == 1);
    $("#button_r2")   .prop("checked", playback_buffer[idx].digital[11] == 1);

    $("#infillsA")     .attr("fill", playback_buffer[idx].digital[0] == 1 ? "#2ca3fc" : "#fff");
    $("#infillsB")     .attr("fill", playback_buffer[idx].digital[1] == 1 ? "#2ca3fc" : "#fff");
    $("#infillsX")     .attr("fill", playback_buffer[idx].digital[2] == 1 ? "#2ca3fc" : "#fff");
    $("#infillsY")     .attr("fill", playback_buffer[idx].digital[3] == 1 ? "#2ca3fc" : "#fff");
    $("#infillsUp")    .attr("fill", playback_buffer[idx].digital[4] == 1 ? "#2ca3fc" : "#fff");
    $("#infillsRight") .attr("fill", playback_buffer[idx].digital[5] == 1 ? "#2ca3fc" : "#fff");
    $("#infillsDown")  .attr("fill", playback_buffer[idx].digital[6] == 1 ? "#2ca3fc" : "#fff");
    $("#infillsLeft")  .attr("fill", playback_buffer[idx].digital[7] == 1 ? "#2ca3fc" : "#fff");
    $("#shoulderL1")   .attr("fill", playback_buffer[idx].digital[8] == 1 ? "#2ca3fc" : "#fff");
    $("#shoulderL2")   .attr("fill", playback_buffer[idx].digital[9] == 1 ? "#2ca3fc" : "#fff");
    $("#shoulderR1")   .attr("fill", playback_buffer[idx].digital[10] == 1 ? "#2ca3fc" : "#fff");
    $("#shoulderR2")   .attr("fill", playback_buffer[idx].digital[11] == 1 ? "#2ca3fc" : "#fff");
    
    $("#axisR").css("transform", "translate(" + (playback_buffer[idx].axis[0] * 0.118) + "px," + (playback_buffer[idx].axis[1] * -0.118) + "px)");
    $("#axisL").css("transform", "translate(" + (playback_buffer[idx].axis[3] * 0.118) + "px," + (playback_buffer[idx].axis[2] * -0.118) + "px)");

    Array.from(document.getElementsByTagName("input")).forEach(item => {
        item.blur();
    });

    /** @type {CanvasRenderingContext2D} */
    var context = document.getElementById("tracker").getContext("2d")
    
    context.clearRect(0, 0, 9999, 9999);

    context.strokeStyle = "#ff0000"
    context.beginPath()
    context.moveTo(context.canvas.width / 2, 0)
    context.lineTo(context.canvas.width / 2, context.canvas.height)
    context.closePath()
    context.stroke()
    /** @type {string} */
    var optionChosen = document.getElementById("inputs").value

    var randomVals = [
        // RIGHT
        "#883300",
        "#b32100",
        // LEFT
        "#885b00",
        "#b36200",

        "#c3c621ff",
        "#64991bff",

        "#19911bff",
        "#1ba26aff",

        "#1ea391ff",
        "#1ea4b3ff",

        "#1e6cb4ff",
        "#316fe1ff",

        "#2d3bff",
        "#6c2dff",

        "#b52dff",
        "#ff2df4",
    ]

    if (optionChosen == "all") {
        for (let axisIdx = 0; axisIdx < 4; axisIdx++) {
            drawCanvas(idx, axisIdx, context, false, randomVals[axisIdx])
        }

        for (let axisIdx = 0; axisIdx < 12; axisIdx++) {
            drawCanvas(idx, axisIdx, context, true, randomVals[axisIdx + 4])
        }
    } else {
        if (optionChosen[0] == 'a') {
            drawCanvas(idx, Number(optionChosen[4]), context, false, randomVals[Number(optionChosen[4])])
        } else {
            drawCanvas(idx, Number(optionChosen.substring(7)) - 4, context, true, randomVals[Number(optionChosen.substring(7))])
        }
    }
};

function drawCanvas(idx, num, context, button, col) {
    context.strokeStyle = col
    var raw, mult
    context.beginPath()
    if (!button) {
        raw = playback_buffer[idx].axis[num]
        mult = 0.9
    } else {
        raw = playback_buffer[idx].digital[num]
        mult = 114.3
    }
    context.moveTo(0 - idx + context.canvas.width / 2, ((((raw * mult) / -127) + 1) / 2) * context.canvas.height)
    for (let index = Math.max(idx - 150, 0); index < Math.min(idx + 150, playback_buffer.length) - 1; index++) {
        if (!button) {
            raw = playback_buffer[index].axis[num]
            mult = 0.9
        } else {
            raw = playback_buffer[index].digital[num]
            mult = 114.3
        }

        const element = (((raw * mult) / -127) + 1) / 2; // normalize to [0, 1]

        context.lineTo((index - idx + context.canvas.width / 2), element * context.canvas.height)
    }
    context.moveTo(0 - idx + context.canvas.width / 2, ((((raw * mult) / -127) + 1) / 2) * context.canvas.height)
    context.closePath();
    context.stroke()
}

function begin_playback(start_pos = 0) {
    if (isRunning) return;
    if (playback_cursor == playback_buffer.length - 1) start_pos = 0;
    fpsInterval = 5;
    playback_cursor = start_pos;
    then = window.performance.now();
    startTime = then;
    console.log(startTime);
    tickPlayback();
    $("#button_stop").text("Pause");
    isRunning = true;
};

function tickPlayback() {
    // stop if user says to stop or if cursor has reached end
    if (stopNextTick || playback_cursor >= playback_buffer.length - 1) {
        stopNextTick = false;
        isRunning = false;
        $("#button_stop").text("Stop");
        return;
    }

    // request another frame
    requestAnimationFrame(tickPlayback);

    // calc (slang for calculate) elapsed time since last loop
    now = window.performance.now();
    elapsed = now - then;
            
    // if enough time has elapsed, draw the next frame
    while (elapsed > fpsInterval) {
        // Get ready for next frame by setting then=now
        then = now - (elapsed % fpsInterval);

        // update stuff
        playback_cursor++;
        update_ui(playback_cursor);
        // stop if user says to stop or if cursor has reached end
        if (playback_cursor >= playback_buffer.length - 1) {
            isRunning = false;
            $("#button_stop").text("Stop");
            return;
        }
        elapsed -= fpsInterval;
    };
};

function saveToBuffer(section, idx, data) {
    let newValue = section === "axis" ? data : (data ? 1 : 0);
    add_undo_action({
        "action": "update_buffer",
        "common_data": [section, idx],
        "old_data": [playback_buffer[playback_cursor][section][idx]],
        "new_data": [newValue],
        "cursor_position": playback_cursor
    });
    playback_buffer[playback_cursor][section][idx] = newValue;
    
    update_ui(playback_cursor);
}

function animate(time) {
    tweens.forEach((val) => {
        val.update(time)
        if (!val.isPlaying()) {
            const index = tweens.indexOf(val);
            if (index > -1) { // only splice array when item is found
                tweens.splice(index, 1); // 2nd parameter means remove one item only
            }
        }
    })
    requestAnimationFrame(animate);
}

window.onload = function () {
    let input = document.getElementById("input");
    input.addEventListener("change", () => {
        // clear out the array in the event of loading a new file if one is already loaded
        if (coords.y == -50) {
            const tween = new Tween(coords, false).to({x: -50, y: 450 }, 1000).easing(Easing.Quadratic.InOut).onUpdate(() => {
                document.getElementById("input_button").style.setProperty("transform", "translate(" + coords.x + "%, " + coords.y + "%)");
            }).start();
            const tween2 = new Tween(alpha, false).to({val: 100}, 1200).easing(Easing.Quadratic.InOut).onUpdate(() => {
                document.getElementById("data_ui").style.setProperty("opacity", alpha.val + "%");
                document.getElementById("controller_ui").style.setProperty("opacity", alpha.val + "%");
            }).start();
            tweens.push(tween);
            tweens.push(tween2);
        }
        playback_buffer.length = 0;
        let reader = new FileReader();
        reader.onload = function () {
            // read out the entire file (kinda like a C io stream but manually implemented)
            let arrayBuffer = this.result, array = new Int8Array(arrayBuffer);
            let cursor = 0;
            let length = array[cursor++];
            // float32 - double (4 bytes to read)
            originPosX = bytes_to_float([array[cursor++], array[cursor++], array[cursor++], array[cursor++]]);
            originPosY = bytes_to_float([array[cursor++], array[cursor++], array[cursor++], array[cursor++]]);
            originHeading = bytes_to_float([array[cursor++], array[cursor++], array[cursor++], array[cursor++]]);
            recordingLength = length;
            while (cursor < array.byteLength) {
                playback_buffer.push({
                    "axis": [array[cursor++], array[cursor++], array[cursor++], array[cursor++]],
                    "digital": [array[cursor++], array[cursor++], array[cursor++], array[cursor++], array[cursor++], array[cursor++], array[cursor++], array[cursor++], array[cursor++], array[cursor++], array[cursor++], array[cursor++]],
                    "gps": {
                        "x": bytes_to_float([array[cursor++], array[cursor++], array[cursor++], array[cursor++]]),
                        "y": bytes_to_float([array[cursor++], array[cursor++], array[cursor++], array[cursor++]]),
                        "heading": bytes_to_float([array[cursor++], array[cursor++], array[cursor++], array[cursor++]])
                    }
                });
            };
            // copy the original to a reserve buffer
            original_playback_buffer = structuredClone(playback_buffer);

            // Make all the UI
            $("#controller_ui").removeClass("hidden");
            $("#data_ui").removeClass("hidden");
            // $("#timeline").css("width", (length * 40) + "px");
            $("#timeline").attr("max", playback_buffer.length - 1);
            $("#progress").val("0");
            $("#max").text("/" + (playback_buffer.length - 1).toString());
            $("#timestamp").text("0.00 / " + recordingLength + "s");
            document.getElementById("timeline").oninput = function (event) {
                playback_cursor = Number(event.target.value);
                update_ui(playback_cursor);
            }
        }
        reader.readAsArrayBuffer(input.files[0]);
    });

    document.getElementById("axis1")        .addEventListener("change", e => { saveToBuffer("axis",     0, e.target.value); e.target.blur() });
    document.getElementById("axis2")        .addEventListener("change", e => { saveToBuffer("axis",     1, e.target.value); e.target.blur() });
    document.getElementById("axis3")        .addEventListener("change", e => { saveToBuffer("axis",     2, e.target.value); e.target.blur() });
    document.getElementById("axis4")        .addEventListener("change", e => { saveToBuffer("axis",     3, e.target.value); e.target.blur() });
    document.getElementById("button_a")     .addEventListener("change", e => { saveToBuffer("digital",  0, e.target.checked); e.target.blur() });
    document.getElementById("button_b")     .addEventListener("change", e => { saveToBuffer("digital",  1, e.target.checked); e.target.blur() });
    document.getElementById("button_x")     .addEventListener("change", e => { saveToBuffer("digital",  2, e.target.checked); e.target.blur() });
    document.getElementById("button_y")     .addEventListener("change", e => { saveToBuffer("digital",  3, e.target.checked); e.target.blur() });
    document.getElementById("button_up")    .addEventListener("change", e => { saveToBuffer("digital",  4, e.target.checked); e.target.blur() });
    document.getElementById("button_right") .addEventListener("change", e => { saveToBuffer("digital",  5, e.target.checked); e.target.blur() });
    document.getElementById("button_down")  .addEventListener("change", e => { saveToBuffer("digital",  6, e.target.checked); e.target.blur() });
    document.getElementById("button_left")  .addEventListener("change", e => { saveToBuffer("digital",  7, e.target.checked); e.target.blur() });
    document.getElementById("button_l1")    .addEventListener("change", e => { saveToBuffer("digital",  8, e.target.checked); e.target.blur() });
    document.getElementById("button_l2")    .addEventListener("change", e => { saveToBuffer("digital",  9, e.target.checked); e.target.blur() });
    document.getElementById("button_r1")    .addEventListener("change", e => { saveToBuffer("digital", 10, e.target.checked); e.target.blur() });
    document.getElementById("button_r2")    .addEventListener("change", e => { saveToBuffer("digital", 11, e.target.checked); e.target.blur() });

    document.getElementById("infillsA")     .addEventListener("click",  e => { document.getElementById("button_a").checked = !document.getElementById("button_a").checked; document.getElementById("button_a").dispatchEvent(new Event("change")) })
    document.getElementById("infillsB")     .addEventListener("click",  e => { document.getElementById("button_b").checked = !document.getElementById("button_b").checked; document.getElementById("button_b").dispatchEvent(new Event("change")) })
    document.getElementById("infillsX")     .addEventListener("click",  e => { document.getElementById("button_x").checked = !document.getElementById("button_x").checked; document.getElementById("button_x").dispatchEvent(new Event("change")) })
    document.getElementById("infillsY")     .addEventListener("click",  e => { document.getElementById("button_y").checked = !document.getElementById("button_y").checked; document.getElementById("button_y").dispatchEvent(new Event("change")) })

    document.getElementById("infillsUp")    .addEventListener("click",  e => { document.getElementById("button_up").checked = !document.getElementById("button_up").checked; document.getElementById("button_up").dispatchEvent(new Event("change")) })
    document.getElementById("infillsDown")  .addEventListener("click",  e => { document.getElementById("button_down").checked = !document.getElementById("button_down").checked; document.getElementById("button_down").dispatchEvent(new Event("change")) })
    document.getElementById("infillsRight") .addEventListener("click",  e => { document.getElementById("button_right").checked = !document.getElementById("button_right").checked; document.getElementById("button_right").dispatchEvent(new Event("change")) })
    document.getElementById("infillsLeft")  .addEventListener("click",  e => { document.getElementById("button_left").checked = !document.getElementById("button_left").checked; document.getElementById("button_left").dispatchEvent(new Event("change")) })

    document.getElementById("shoulderL1")     .addEventListener("click",  e => { document.getElementById("button_l1").checked = !document.getElementById("button_l1").checked; document.getElementById("button_l1").dispatchEvent(new Event("change")) })
    document.getElementById("shoulderL2")     .addEventListener("click",  e => { document.getElementById("button_l2").checked = !document.getElementById("button_l2").checked; document.getElementById("button_l2").dispatchEvent(new Event("change")) })
    document.getElementById("shoulderR1")     .addEventListener("click",  e => { document.getElementById("button_r1").checked = !document.getElementById("button_r1").checked; document.getElementById("button_r1").dispatchEvent(new Event("change")) })
    document.getElementById("shoulderR2")     .addEventListener("click",  e => { document.getElementById("button_r2").checked = !document.getElementById("button_r2").checked; document.getElementById("button_r2").dispatchEvent(new Event("change")) })
    
    document.getElementById("timeline")     .addEventListener("mouseup", e => {
        e.preventDefault();
        add_undo_action({
            "action": "move_cursor",
            "common_data": [],
            "old_data": [Number(previous_position)],
            "new_data": [Number(playback_cursor)],
            "cursor_position": playback_cursor
        })
        previous_position = playback_cursor;
    });

    document.getElementById("inputs")       .addEventListener("change", e => { update_ui(playback_cursor) })

    requestAnimationFrame(animate);
}

window.onkeydown = function (evt) {
    // ctrl+shift+z or ctrl+y
    if ((evt.key === 'Z' && (evt.ctrlKey || evt.metaKey) && evt.shiftKey) || (evt.key === 'y' && (evt.ctrlKey || evt.metaKey))) {
        evt.preventDefault();
        // handle redo action
        redo();
    //ctrl+z
    } else if (evt.key === 'z' && (evt.ctrlKey || evt.metaKey)) {
        evt.preventDefault();
        // handle undo action
        undo();
    }
}