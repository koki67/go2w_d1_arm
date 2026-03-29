#!/usr/bin/env python3

from pathlib import Path
import sys


def apply_replacements(text: str) -> str:
    replacements = [
        (
            """    explicit DdsTopic(const DdsParticipantPtr& participant, const std::string& name, const DdsTopicQos& qos) :
        mNative(__UT_DDS_NULL__)
    {
        UT_DDS_EXCEPTION_TRY

        auto topicQos = participant->GetNative().default_topic_qos();
        qos.CopyToNativeQos(topicQos);

        mNative = NATIVE_TYPE(participant->GetNative(), name, topicQos);

        UT_DDS_EXCEPTION_CATCH(mLogger, true)
    }

    ~DdsTopic()
    {
        mNative = __UT_DDS_NULL__;
    }""",
            """    explicit DdsTopic(const DdsParticipantPtr& participant, const std::string& name, const DdsTopicQos& qos) :
        mNative([&]() {
            auto topicQos = participant->GetNative().default_topic_qos();
            qos.CopyToNativeQos(topicQos);
            return NATIVE_TYPE(participant->GetNative(), name, topicQos);
        }())
    {
    }

    ~DdsTopic() = default;""",
        ),
        (
            """    explicit DdsWriter(const DdsPublisherPtr publisher, const DdsTopicPtr<MSG>& topic, const DdsWriterQos& qos) :
        mNative(__UT_DDS_NULL__)
    {
        UT_DDS_EXCEPTION_TRY

        auto writerQos = publisher->GetNative().default_datawriter_qos();
        qos.CopyToNativeQos(writerQos);

        mNative = NATIVE_TYPE(publisher->GetNative(), topic->GetNative(), writerQos);

        UT_DDS_EXCEPTION_CATCH(mLogger, true)
    }

    ~DdsWriter()
    {
        mNative = __UT_DDS_NULL__;
    }""",
            """    explicit DdsWriter(const DdsPublisherPtr publisher, const DdsTopicPtr<MSG>& topic, const DdsWriterQos& qos) :
        mNative([&]() {
            auto writerQos = publisher->GetNative().default_datawriter_qos();
            qos.CopyToNativeQos(writerQos);
            return NATIVE_TYPE(publisher->GetNative(), topic->GetNative(), writerQos);
        }())
    {
    }

    ~DdsWriter() = default;""",
        ),
        (
            """    explicit DdsReader(const DdsSubscriberPtr& subscriber, const DdsTopicPtr<MSG>& topic, const DdsReaderQos& qos) :
        mNative(__UT_DDS_NULL__)
    {
        UT_DDS_EXCEPTION_TRY

        auto readerQos = subscriber->GetNative().default_datareader_qos();
        qos.CopyToNativeQos(readerQos);

        mNative = NATIVE_TYPE(subscriber->GetNative(), topic->GetNative(), readerQos);

        UT_DDS_EXCEPTION_CATCH(mLogger, true)
    }

    ~DdsReader()
    {
        mNative = __UT_DDS_NULL__;
    }""",
            """    explicit DdsReader(const DdsSubscriberPtr& subscriber, const DdsTopicPtr<MSG>& topic, const DdsReaderQos& qos) :
        mNative([&]() {
            auto readerQos = subscriber->GetNative().default_datareader_qos();
            qos.CopyToNativeQos(readerQos);
            return NATIVE_TYPE(subscriber->GetNative(), topic->GetNative(), readerQos);
        }())
    {
    }

    ~DdsReader() = default;""",
        ),
    ]

    updated = text
    for old, new in replacements:
        if old not in updated:
            raise RuntimeError(
                f"expected SDK block not found for replacement starting with: {old.splitlines()[0].strip()}"
            )
        updated = updated.replace(old, new, 1)
    return updated


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: patch_unitree_sdk2_dds_entity.py <path>", file=sys.stderr)
        return 2

    path = Path(sys.argv[1])
    original = path.read_text()
    updated = apply_replacements(original)
    path.write_text(updated)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
