'''*****************************************************************************************************
# FileName    : .py
# FileFunction: 总线测试程序
# Comments    :
*****************************************************************************************************'''
from canlib import canlib, Frame

def main(args=None):
    '''*****************************************************************************************************
    - FunctionName: main
    - Function    : 总线测试程序
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    *****************************************************************************************************'''
    num_channels = canlib.getNumberOfChannels()
    print("Found %d channels" % num_channels)
    for ch in range(0, num_channels):
        print("%d. %s (%s / %s)" % (
            ch, canlib.ChannelData(ch).channel_name,
            canlib.ChannelData(ch).card_upc_no,
            canlib.ChannelData(ch).card_serial_no))
        
    print("canlib dll version:", canlib.dllversion())

    ch0 = setUpChannel(channel=0)
    ch1 = setUpChannel(channel=1)

    frame = Frame(id_=100, data=[1, 2, 3, 4], flags=canlib.MessageFlag.EXT)
    ch1.write(frame)

    while True:
        try:
            frame = ch0.read()
            print(frame)
            break
        except (canlib.canNoMsg) as ex:
            pass
        except (canlib.canError) as ex:
            print(ex)

    tearDownChannel(ch0)
    tearDownChannel(ch1)

def setUpChannel(channel=0,
                 openFlags=canlib.Open.ACCEPT_VIRTUAL,
                 bitrate=canlib.canBITRATE_500K,
                 outputControl=canlib.Driver.NORMAL):
    ch = canlib.openChannel(channel, openFlags)
    print("Using channel: %s, EAN: %s" % (
        canlib.ChannelData(channel).device_name,
        canlib.ChannelData(channel).card_upc_no))
    ch.setBusOutputControl(outputControl)
    ch.setBusParams(bitrate)
    ch.busOn()
    return ch

def tearDownChannel(ch):
    ch.busOff()
    ch.close()

if __name__ == '__main__':
    main()