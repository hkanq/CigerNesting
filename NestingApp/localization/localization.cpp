#include "localization/localization.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

#include <array>
#include <cstddef>

namespace nest {
namespace {

constexpr size_t textIndex(TextId id) {
    return static_cast<size_t>(id);
}

using TextTable = std::array<const wchar_t*, textIndex(TextId::Count)>;

constexpr TextTable turkishTexts{
    L"CigerNesting",
    L"Dosya Aç",
    L"Kaydet",
    L"Başlat",
    L"Durdur",
    L"Corel Bağlantısı",
    L"Corel'e Aktar",
    L"Ayarlar",
    L"Dizim Ayarları",
    L"Sac Genişliği",
    L"Sac Yüksekliği",
    L"Parça Aralığı",
    L"Kenar Boşluğu",
    L"Rotasyon Açık",
    L"Rotasyon Modu",
    L"Açı Hassasiyeti",
    L"Aynalama Açık",
    L"Kalite Modu",
    L"Süre Limiti (sn)",
    L"Thread Sayısı (0=Otomatik)",
    L"Faz",
    L"Çarpışma",
    L"Doluluk",
    L"Dosya Aç",
    L"İçe aktarma başarısız",
    L"İçe aktarma başarılı",
    L"Önce SVG, DXF veya PLT/HPGL dosyası açın.",
    L"Corel bridge mimarisi hazır. Makro kurulumu bu aşamada bilinçli olarak pasif.",
    L"Kaydet altyapısı hazır; dosya formatı sonraki adımda netleştirilecek.",
    L"Boşta",
    L"Geometri Hazırlanıyor",
    L"İlk Yerleşim",
    L"Keşif",
    L"Çarpışma Çözümü",
    L"Sıkıştırma",
    L"Ultra İnce Ayar",
    L"Son Doğrulama",
    L"Tamamlandı",
    L"Durduruldu",
    L"Hızlı",
    L"Dengeli",
    L"Maksimum Kalite",
    L"Yok",
    L"Dik Açılar",
    L"45 Derece",
    L"Sabit Adım",
    L"Sürekli İnce Ayar",
    L"Desteklenen Dosyalar",
    L"SVG Dosyaları",
    L"DXF Dosyaları",
    L"PLT/HPGL Dosyaları",
    L"Tüm Dosyalar",
    L"Dosya Paneli",
    L"Parça Sayısı",
    L"Dosya yüklenmedi",
    L"SVG, PLT/HPGL veya DXF açın. Mouse tekerleği zoom, sol sürükleme pan.",
    L"Başlamak için dosya açın",
    L"Corel export session modeli üretildi. Gerçek Corel makro/pipe aktarımı sonraki entegrasyon adımında bağlanacak."
};

constexpr TextTable englishTexts{
    L"CigerNesting",
    L"Open File",
    L"Save",
    L"Start",
    L"Stop",
    L"Corel Connection",
    L"Export to Corel",
    L"Settings",
    L"Nesting Settings",
    L"Sheet Width",
    L"Sheet Height",
    L"Part Spacing",
    L"Margin",
    L"Rotation Enabled",
    L"Rotation Mode",
    L"Angle Precision",
    L"Mirroring Enabled",
    L"Quality Mode",
    L"Time Limit (s)",
    L"Thread Count (0=Auto)",
    L"Phase",
    L"Collision",
    L"Utilization",
    L"Open File",
    L"Import failed",
    L"Import successful",
    L"Open an SVG, DXF, or PLT/HPGL file first.",
    L"Corel bridge architecture is ready. Macro installation is intentionally inactive in this stage.",
    L"Save infrastructure is ready; the file format will be finalized in a later step.",
    L"Idle",
    L"Prepare Geometry",
    L"Initial Placement",
    L"Exploration",
    L"Collision Resolution",
    L"Compression",
    L"Ultra Refinement",
    L"Final Validation",
    L"Done",
    L"Stopped",
    L"Fast",
    L"Balanced",
    L"Max Quality",
    L"None",
    L"Right Angles",
    L"45 Degrees",
    L"Fixed Step",
    L"Continuous Refine",
    L"Supported Files",
    L"SVG Files",
    L"DXF Files",
    L"PLT/HPGL Files",
    L"All Files",
    L"File Panel",
    L"Part Count",
    L"No file loaded",
    L"Open SVG, PLT/HPGL, or DXF. Mouse wheel zooms, left drag pans.",
    L"Open a file to begin",
    L"Corel export session model was generated. Real Corel macro/pipe transfer will be connected in a later integration step."
};

} // namespace

Localization& Localization::instance() {
    static Localization localization;
    return localization;
}

Localization::Localization() {
    setLanguage(Language::Auto);
}

void Localization::setLanguage(Language language) {
    requestedLanguage_ = language;
    activeLanguage_ = language == Language::Auto ? detectSystemLanguage() : language;
    if (activeLanguage_ == Language::Auto) {
        activeLanguage_ = Language::English;
    }
}

Language Localization::currentLanguage() const {
    return activeLanguage_;
}

Language Localization::detectSystemLanguage() {
#ifdef _WIN32
    const LANGID langId = GetUserDefaultUILanguage();
    return PRIMARYLANGID(langId) == LANG_TURKISH ? Language::Turkish : Language::English;
#else
    return Language::English;
#endif
}

const wchar_t* Localization::text(TextId id) const {
    const size_t index = textIndex(id);
    if (index >= textIndex(TextId::Count)) {
        return L"";
    }
    return activeLanguage_ == Language::Turkish ? turkishTexts[index] : englishTexts[index];
}

} // namespace nest
