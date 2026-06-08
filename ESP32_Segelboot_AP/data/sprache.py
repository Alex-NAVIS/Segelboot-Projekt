import os
import re
import json
from html.parser import HTMLParser

# Pfade definieren
start_dir = os.path.dirname(os.path.abspath(__file__))
output_dir = os.path.join(start_dir, "Sprache")
output_js_dir = os.path.join(output_dir, "js")
lang_file_path = os.path.join(output_js_dir, "lang.js")

# Ordner erstellen falls nicht vorhanden
os.makedirs(output_js_dir, exist_ok=True)

# Regulärer Ausdruck für zu ignorierende Messwerte/Zahlen/Einheiten
IGNORE_REGEX = re.compile(r"^([0-9\s\.\,\:\-\°\%\+\/\d\?]+|V|A|Ah|W|kW|kn|km/h|m/s|hPa|°C|°F|%|NaN|true|false|null|undefined|&nbsp;)$", re.IGNORECASE)

# Globale Übersetzungsdatenbank
translations = {"de": {}, "en": {}, "fr": {}, "es": {}}

# Mini-Wörterbuch für maritime Begriffe
DICTIONARY = {
    "autopilot": {"en": "Autopilot", "fr": "Pilote automatique", "es": "Piloto automático"},
    "einstellungen": {"en": "Settings", "fr": "Réglages", "es": "Ajustes"},
    "system-einstellungen": {"en": "System Settings", "fr": "Réglages système", "es": "Ajustes del sistema"},
    "autopilot-einstellungen": {"en": "Autopilot Settings", "fr": "Réglages pilote", "es": "Ajustes del piloto"},
    "speichern": {"en": "Save", "fr": "Enregistrer", "es": "Guardar"},
    "abbrechen": {"en": "Cancel", "fr": "Annuler", "es": "Cancelar"},
    "löschen": {"en": "Delete", "fr": "Supprimer", "es": "Eliminar"},
    "startseite": {"en": "Home", "fr": "Accueil", "es": "Inicio"},
    "karte": {"en": "Map", "fr": "Carte", "es": "Mapa"},
    "kompass": {"en": "Compass", "fr": "Compas", "es": "Compás"},
    "künstlicher horizont": {"en": "Artificial Horizon", "fr": "Horizon artificiel", "es": "Horizonte artificial"},
    "logbuch": {"en": "Logbook", "fr": "Livre de bord", "es": "Cuaderno de bitácora"},
    "wetterdaten": {"en": "Weather Data", "fr": "Météo", "es": "Datos meteorológicos"},
    "tidenkalender": {"en": "Tide Table", "fr": "Marées", "es": "Mareas"},
    "alarme": {"en": "Alarms", "fr": "Alarmes", "es": "Alarmas"},
    "datei-upload": {"en": "File Upload", "fr": "Mise en ligne", "es": "Cargar archivo"},
    "windgeschwindigkeit": {"en": "Wind Speed", "fr": "Vitesse du vent", "es": "Velocidad del viento"},
    "windrichtung": {"en": "Wind Direction", "fr": "Direction du vent", "es": "Dirección del viento"},
    "kurs (cog)": {"en": "Course (COG)", "fr": "Cap (COG)", "es": "Rumbo (COG)"},
    "soll-kurs": {"en": "Target Heading", "fr": "Cap visé", "es": "Rumbo fijado"},
    "ruderlage": {"en": "Rudder Angle", "fr": "Angle de barre", "es": "Ángulo de timón"}
}

def get_translation(text, lang):
    clean = text.lower().strip()
    if clean in DICTIONARY and lang in DICTIONARY[clean]:
        return DICTIONARY[clean][lang]
    return f"[{lang.upper()}] {text}"

class HTMLI18nProcessor(HTMLParser):
    def __init__(self, file_prefix):
        super().__init__()
        self.file_prefix = file_prefix
        self.result_html = ""
        self.text_idx = 1
        self.placeholder_idx = 1
        self.tag_stack = []
        self.in_skip_tag = False

    def handle_starttag(self, tag, attrs):
        self.tag_stack.append(tag)
        attrs_dict = dict(attrs)
        
        if tag in ['script', 'style']:
            self.in_skip_tag = True

        # Injektion der lang.js im Head
        if tag == 'head':
            self.result_html += "<head><script src=\"js/lang.js\"></script>"
            return

        # Verarbeite Placeholders
        if 'placeholder' in attrs_dict and not IGNORE_REGEX.match(attrs_dict['placeholder']):
            raw_text = attrs_dict['placeholder'].strip()
            key = f"{self.file_prefix}_placeholder_{self.placeholder_idx}"
            self.placeholder_idx += 1
            
            for lang in ["de", "en", "fr", "es"]:
                translations[lang][key] = raw_text if lang == "de" else get_translation(raw_text, lang)
            
            attrs_dict['data-i18n-placeholder'] = key

        # Baue Tag wieder zusammen
        attr_str = ""
        for k, v in attrs_dict.items():
            attr_str += f' {k}="{v}"' if v is not None else f' {k}'
        
        self.result_html += f"<{tag}{attr_str}>"

    def handle_endtag(self, tag):
        if self.tag_stack and self.tag_stack[-1] == tag:
            self.tag_stack.pop()
        if tag in ['script', 'style']:
            self.in_skip_tag = False
        
        if tag == 'head':
            return # Bereits beim Starttag abgehandelt
            
        self.result_html += f"</{tag}>"

    def handle_data(self, data):
        raw_text = data.strip()
        
        # Prüfen ob der Text übersetzt werden muss
        if raw_text and not self.in_skip_tag and not IGNORE_REGEX.match(raw_text):
            key = f"{self.file_prefix}_txt_{self.text_idx}"
            self.text_idx += 1
            
            for lang in ["de", "en", "fr", "es"]:
                translations[lang][key] = raw_text if lang == "de" else get_translation(raw_text, lang)
            
            # Attribut nachträglich in das letzte geöffnete Tag einfügen
            if self.result_html.endswith(">") and not self.result_html.endswith("/>"):
                self.result_html = self.result_html[:-1] + f' data-i18n="{key}">'
                
        self.result_html += data

print(f"Starte Python-Extraktion im Ordner: {start_dir}\n")

# Alle HTML-Dateien im Ordner verarbeiten
for file_name in os.listdir(start_dir):
    if not file_name.lower().endswith('.html'):
        continue
        
    file_path = os.path.join(start_dir, file_name)
    file_prefix = os.path.splitext(file_name)[0].lower()
    
    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
        html_content = f.read()
        
    processor = HTMLI18nProcessor(file_prefix)
    processor.feed(html_content)
    
    # Modifiziertes HTML im neuen Ordner abspeichern
    output_file_path = os.path.join(output_dir, file_name)
    with open(output_file_path, 'w', encoding='utf-8') as f:
        f.write(processor.result_html)
        
    print(f" -> Erstellt: Sprache/{file_name}")

# Die zentrale lang.js schreiben
js_content = f"""// Automatisch generiert durch i18n_builder.py
const translations = {json.dumps(translations, indent=4, ensure_ascii=False)};

let currentLang = localStorage.getItem('selectedLang') || navigator.language.slice(0, 2) || 'de';
if (!translations[currentLang]) currentLang = 'de';

function applyTranslations() {{
    document.documentElement.lang = currentLang;
    
    document.querySelectorAll('[data-i18n]').forEach(el => {{
        const key = el.getAttribute('data-i18n');
        if (translations[currentLang][key]) {{
            el.innerText = translations[currentLang][key];
        }}
    }});

    document.querySelectorAll('[data-i18n-placeholder]').forEach(el => {{
        const key = el.getAttribute('data-i18n-placeholder');
        if (translations[currentLang][key]) {{
            el.setAttribute('placeholder', translations[currentLang][key]);
        }}
    }});
}}

function changeLanguage(lang) {{
    if (translations[lang]) {{
        currentLang = lang;
        localStorage.setItem('selectedLang', lang);
        applyTranslations();
    }}
}}

document.addEventListener('DOMContentLoaded', applyTranslations);
"""

with open(lang_file_path, 'w', encoding='utf-8') as f:
    f.write(js_content)

print(f"\n▶ ZENTRALE DATEI ERSTELLT: Sprache/js/lang.js")
print("Fertig! Deine Originaldateien wurden nicht verändert.")
