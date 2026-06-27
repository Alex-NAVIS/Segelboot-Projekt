/* ============================================================
   NAVIS CENTRAL NAVBAR MANAGER WITH LANGUAGE SELECT
   ============================================================ */

/**
 * Erzeugt die Navbar dynamisch inklusive Sprachauswahl.
 * @param {Array<string>} activeButtons - IDs der Seiten, die angezeigt werden sollen (Leer = Alle Seiten).
 */
function createNavbar(activeButtons = []) {
    const oldNavbar = document.getElementById("navbar");
    if (oldNavbar) oldNavbar.remove();

    const navbar = document.createElement("div");
    navbar.id = "navbar";

    const allPages = [
        { id: "dashboard", href: "index.html", text: "Dashboard" },
        { id: "karte", href: "karte.html", text: "Karte" },
        { id: "kompass", href: "kompass.html", text: "Kompass" },
        { id: "autopilot", href: "autopilot.html", text: "Autopilot" },
        { id: "horizont", href: "horizont.html", text: "Horizont" },
        { id: "polar", href: "polar.html", text: "Polar" },
        { id: "alarm", href: "Alarm.html", text: "Überwachung" },
        { id: "logbuch", href: "logbuch.html", text: "Logbuch" },
        { id: "system", href: "system_settings.html", text: "System" },
        { id: "upload", href: "upload.html", text: "Upload" }
    ];

    const pagesToRender = activeButtons.length > 0 
        ? allPages.filter(page => activeButtons.includes(page.id))
        : allPages;

    let linksHtml = `
        <div id="wsStatus">
            <div id="wsLed"></div>
            <span>WS</span>
        </div>
    `;

    pagesToRender.forEach(page => {
        linksHtml += `<a href="${page.href}" class="navbtn" id="nav-${page.id}" data-i18n="nav_${page.id}">${page.text}</a>`;
    });

    // NEU: Klar zugeordnete data-i18n Attribute (lang_de, lang_en, etc.)
    linksHtml += `
        <div class="lang-select-container" style="padding-left: 5px;">
            <select id="langSelect">
                <option value="de" data-i18n="lang_de">Deutsch</option>
                <option value="en" data-i18n="lang_en">English</option>
                <option value="fr" data-i18n="lang_fr">Français</option>
                <option value="it" data-i18n="lang_it">Italiano</option>
                <option value="es" data-i18n="lang_es">Español</option>
                <option value="da" data-i18n="lang_da">Dansk</option>
                <option value="nl" data-i18n="lang_nl">Nederlands</option>
                <option value="no" data-i18n="lang_no">Norsk</option>
                <option value="sv" data-i18n="lang_sv">Svenska</option>
                <option value="ru" data-i18n="lang_ru">Русский</option>
                <option value="pl" data-i18n="lang_pl">Polski</option>
                <option value="tr" data-i18n="lang_tr">Türkçe</option>
                <option value="el" data-i18n="lang_el">Ελληνικά</option>
            </select>
        </div>
    `;

    navbar.innerHTML = linksHtml;
    document.body.prepend(navbar);

    const currentPath = window.location.pathname.split("/").pop() || "index.html";
    const activeLink = navbar.querySelector(`.navbtn[href="${currentPath}"]`);
    if (activeLink) {
        activeLink.classList.add("active");
    }

    const selectEl = document.getElementById("langSelect");
    if (selectEl) {
        const savedLang = localStorage.getItem('selectedLang') || navigator.language.slice(0, 2) || 'de';
        selectEl.value = savedLang;

        selectEl.addEventListener("change", (e) => {
            const chosenLang = e.target.value;
            if (typeof changeLanguage === "function") {
                changeLanguage(chosenLang);
            } else {
                console.warn("changeLanguage() wurde noch nicht geladen.");
            }
        });
    }
}
