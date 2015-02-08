

#ifndef WebPageSerializerClient_h
#define WebPageSerializerClient_h

namespace WebKit {
class WebCString;
class WebURL;

// This class is used for providing sink interface that can be used to receive
// the individual chunks of data to be saved.
class WebPageSerializerClient {
public:
    // This enum indicates  This sink interface can receive the individual chunks
    // of serialized data to be saved, so we use values of following enum
    // definition to indicate the serialization status of serializing all html
    // content. If current frame is not complete serialized, call
    // didSerializeDataForFrame with URL of current frame, data, data length and
    // flag CurrentFrameIsNotFinished.
    // If current frame is complete serialized, call didSerializeDataForFrame
    // with URL of current frame, data, data length and flag
    // CurrentFrameIsFinished.
    // If all frames of page are complete serialized, call
    // didSerializeDataForFrame with empty URL, empty data, 0 and flag
    // AllFramesAreFinished.
    enum PageSerializationStatus {
        CurrentFrameIsNotFinished,
        CurrentFrameIsFinished,
        AllFramesAreFinished,
    };

    // Receive the individual chunks of serialized and encoded data to be saved.
    // The parameter frameURL specifies what frame the data belongs. The
    // parameter data contains the available data for saving. The parameter
    // status indicates the status of data serialization.
    virtual void didSerializeDataForFrame(const WebURL& frameURL,
                                          const WebCString& data,
                                          PageSerializationStatus status) = 0;
    WebPageSerializerClient() { }

protected:
    virtual ~WebPageSerializerClient() { }
};

}  // namespace WebKit

#endif
