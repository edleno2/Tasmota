def startmdns()
    import mdns
    print("autoexec.be: starting mdns scale service when wifi connects")
    mdns.start(nil)
    mdns.add_service("_edleno_homeautomation", "_tcp", 9999, {"DeviceType":"Scale"})
end

def postHttp(weight, pitch)
    import string
    var wc = webclient()
    var work = tasmota.cmd("mem3", true)
    var url = work["Mem3"]
    if url == ""
        return true
    end
    work = tasmota.cmd("mem4", true)
    var deviceId = work["Mem4"]
    if deviceId == ""
        return true
    end
    wc.begin(url)
    var postData = string.format("{\"Weight\":\"%d\", \"Pitch\":\"%4.2f\", \"deviceId\":\"%s\"}", int(weight), real(pitch), deviceId)
    wc.add_header("Content-type","application/json")
    var rc = wc.POST(postData)
    # print(rc)
    var response = wc.get_string()
    # print("postHttp: POST to client. Response:")
    # print (response)
end

def gotMqtt (topic, idx, payload)  # callback when mqtt topic tele/[device name]/SENSOR message is received by the mqtt broker and sent back to the current device
    import json
    # print ("gotMqtt: topic: ")
    # print (topic)
    # print ("payload: ")
    # print (payload)
    var mapLoad = json.load(payload)
    if mapLoad.contains("HX711") == false 
        return true
    end

    # print("Weight: ")
    # print(mapLoad["HX711"]["Weight"])

    postHttp(mapLoad["HX711"]["Weight"], mapLoad["HX711"]["Pitch"])
    return true
end


def setupMqtt()
    import mqtt 
    import string
    var statMap = tasmota.cmd("status", true)  # get the general status
    # print("status Topic: ")
    # print(statMap["Status"]["Topic"])
    mqtt.subscribe(string.format("tele/%s/SENSOR",statMap["Status"]["Topic"]), gotMqtt)  # get mqtt bounce backs for topic tele/[device name]/SENSOR
end

def restartDevice()
    tasmota.cmd("restart 1", false)
end

def resetScale()
    tasmota.cmd("sensor34 1", true)
end

def hourlyMaint()
    tasmota.cmd("status 0", true)
end

tasmota.add_rule("Wifi#Connected", startmdns)  # start mdns after the wifi network is connected
tasmota.add_rule("Mqtt#Connected", setupMqtt)  # watch for messages once MQTT is started
#tasmota.add_cron("0 0 1 * * *", restartDevice, "restartDevice_cron")
#tasmota.add_cron("0 0 2-5 * * *", resetScale, "resetScale_cron")
tasmota.add_cron("0 0 * * * *",hourlyMaint, "hourlyMaint_cron")