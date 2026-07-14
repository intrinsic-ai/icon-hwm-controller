#include "icon/utils/log.h"

#include <source_location>
#include <string>
#include <string_view>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "icon/utils/mock_log_sink.h"

namespace intrinsic::log {

using ::testing::ElementsAre;
using ::testing::Eq;
using ::testing::Field;

TEST(Logger, CallsSink) {
  std::vector<LogEntryWithStorage> entries;
  Logger l(Logger::Severity::kDebug, MockSink(entries));
  std::source_location loc = std::source_location::current();
  std::string verb = "walk";
  l.Log(loc, Logger::Severity::kWarning, "Eyyy, I'm {:s}ing here!", verb);

  EXPECT_EQ(entries.size(), 1);
  const auto& e = entries.front();
  EXPECT_EQ(e.msg, "Eyyy, I'm walking here!");
  EXPECT_EQ(e.severity, Logger::Severity::kWarning);
  EXPECT_EQ(e.loc.file_name(), loc.file_name());
  EXPECT_EQ(e.loc.line(), loc.line());
  EXPECT_EQ(e.loc.column(), loc.column());
  EXPECT_EQ(e.loc.function_name(), loc.function_name());
}

TEST(Logger, LogsStringWithSize) {
  constexpr std::string_view kLongString = "lorem ipsum dolor sit amet";
  std::vector<LogEntryWithStorage> entries;
  Logger l(Logger::Severity::kDebug, MockSink(entries));
  std::source_location loc = std::source_location::current();
  l.Log(loc, Logger::Severity::kWarning, "Just a bit of lipsum: {:.{}s}",
        kLongString, 5);

  EXPECT_EQ(entries.size(), 1);
  const auto& e = entries.front();
  EXPECT_EQ(e.msg, "Just a bit of lipsum: lorem");
  EXPECT_EQ(e.severity, Logger::Severity::kWarning);
  EXPECT_EQ(e.loc.file_name(), loc.file_name());
  EXPECT_EQ(e.loc.line(), loc.line());
  EXPECT_EQ(e.loc.column(), loc.column());
  EXPECT_EQ(e.loc.function_name(), loc.function_name());
}

TEST(Logger, DoesNotCallSinkIfLogEntryLevelIsLow) {
  std::vector<LogEntryWithStorage> entries;
  Logger l(Logger::Severity::kError, MockSink(entries));
  l.Log(std::source_location::current(), Logger::Severity::kWarning,
        "You shouldn't see me");

  EXPECT_EQ(entries.size(), 0);
}

TEST(LogMacro, WorksWithReference) {
  std::vector<LogEntryWithStorage> entries;
  Logger l(Logger::Severity::kDebug, MockSink(entries));
  INTRINSIC_SHARED_MEMORY_LOG(DEBUG, l, "Hello Debug!");
  INTRINSIC_SHARED_MEMORY_LOG(INFO, l, "Hello Info!");
  INTRINSIC_SHARED_MEMORY_LOG(WARNING, l, "Hello Warning!");
  INTRINSIC_SHARED_MEMORY_LOG(ERROR, l, "Hello Error!");
  EXPECT_THAT(
      entries,
      ElementsAre(Field("msg", &LogEntryWithStorage::msg, Eq("Hello Debug!")),
                  Field("msg", &LogEntryWithStorage::msg, Eq("Hello Info!")),
                  Field("msg", &LogEntryWithStorage::msg, Eq("Hello Warning!")),
                  Field("msg", &LogEntryWithStorage::msg, Eq("Hello Error!"))));
  // EXPECT_DEATH forks off, so the message logged in there doesn't end up in
  // `entries`.
  EXPECT_DEATH({ INTRINSIC_SHARED_MEMORY_LOG(FATAL, l, "Hello!"); }, "");
}

TEST(LogMacro, WorksWithPointer) {
  std::vector<LogEntryWithStorage> entries;
  Logger l(Logger::Severity::kDebug, MockSink(entries));
  INTRINSIC_SHARED_MEMORY_LOG(DEBUG, &l, "Hello Debug!");
  INTRINSIC_SHARED_MEMORY_LOG(INFO, &l, "Hello Info!");
  INTRINSIC_SHARED_MEMORY_LOG(WARNING, &l, "Hello Warning!");
  INTRINSIC_SHARED_MEMORY_LOG(ERROR, &l, "Hello Error!");

  EXPECT_THAT(
      entries,
      ElementsAre(Field("msg", &LogEntryWithStorage::msg, Eq("Hello Debug!")),
                  Field("msg", &LogEntryWithStorage::msg, Eq("Hello Info!")),
                  Field("msg", &LogEntryWithStorage::msg, Eq("Hello Warning!")),
                  Field("msg", &LogEntryWithStorage::msg, Eq("Hello Error!"))));
  // EXPECT_DEATH forks off, so the message logged in there doesn't end up in
  // `entries`.
  EXPECT_DEATH({ INTRINSIC_SHARED_MEMORY_LOG(FATAL, &l, "Hello!"); }, "");
}

TEST(LogMacro, SkipsLoggingWithNullptr) {
  INTRINSIC_SHARED_MEMORY_LOG(DEBUG, nullptr, "Hello!");
  INTRINSIC_SHARED_MEMORY_LOG(INFO, nullptr, "Hello!");
  INTRINSIC_SHARED_MEMORY_LOG(WARNING, nullptr, "Hello!");
  INTRINSIC_SHARED_MEMORY_LOG(ERROR, nullptr, "Hello!");
  // Even if `logger` is a nullptr, a fatal log should kill the process.
  EXPECT_DEATH({ INTRINSIC_SHARED_MEMORY_LOG(FATAL, nullptr, "Hello!"); }, "");
}

}  // namespace intrinsic::log
