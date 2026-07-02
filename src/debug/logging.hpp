#ifndef DEBUG_LOGGING_HPP
#define DEBUG_LOGGING_HPP

#include <iosfwd>

/// \brief Internal debug logging utilities.
///
/// Categories are enabled in Debug builds through the `CGEOM_DEBUG` environment variable,
/// using the generated helper name: `CGEOM_DEBUG=dcel`, `CGEOM_DEBUG=dcel,rayQuery`, or
/// `CGEOM_DEBUG=all`. Category names are matched case-insensitively.
///
/// Use the generated `debug::<category>Enabled()` helpers to guard debug-only work:
/// \code
/// if (debug::rayQueryEnabled()) {
///     debug::rayQuery() << "query " << point.toString() << '\n';
/// }
/// \endcode
/// In Release builds, the enabled helpers are inline functions that return `false`, so
/// optimized builds should remove guarded debug blocks entirely. Prefer guarding any block
/// that builds strings, walks containers, or performs other non-trivial diagnostic work.
///
/// The generated `debug::<category>()` stream helpers return `std::cerr` when enabled and a
/// sink stream otherwise. They are useful for simple messages, but they do not prevent
/// expensive `operator<<` arguments from being evaluated.
namespace debug {

/// \brief List of debug categories.
///
/// `X(category)` generates:
/// - `debug::categoryEnabled()`
/// - `debug::category()`
#define DEBUG_CATEGORY_LIST(X)                                                                     \
    X(assemble)                                                                                    \
    X(dcel)                                                                                        \
    X(overlay)                                                                                     \
    X(segmentIntersection)                                                                         \
    X(rayQuery)

#ifdef NDEBUG
#define X(category)                                                                                \
    /** \brief Runtime category check implementation used by Debug builds. */                      \
    bool category##EnabledImpl();                                                                  \
    /** \brief Return whether this category is enabled. Always false in Release builds. */         \
    inline bool category##Enabled() {                                                              \
        return false;                                                                              \
    }                                                                                              \
    /** \brief Return the diagnostic stream for this category or a sink stream if disabled. */     \
    std::ostream& category();
#else
#define X(category)                                                                                \
    /** \brief Runtime category check implementation used by Debug builds. */                      \
    bool category##EnabledImpl();                                                                  \
    /** \brief Return whether this category is enabled by `CGEOM_DEBUG`. */                        \
    inline bool category##Enabled() {                                                              \
        return category##EnabledImpl();                                                            \
    }                                                                                              \
    /** \brief Return the diagnostic stream for this category or a sink stream if disabled. */     \
    std::ostream& category();
#endif
DEBUG_CATEGORY_LIST(X)
#undef X

} // namespace debug

#endif // DEBUG_LOGGING_HPP
