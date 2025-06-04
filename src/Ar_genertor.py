#!/usr/bin/env python3
import cv2
import numpy as np
import cv2.aruco as aruco
import os

def generate_aruco_markers():
    """
    Genererer ArUco-markører for robotboks-prosjektet
    """
    # ArUco dictionary - 4x4 betyr 4x4 ruter inne i markøren
    # Støtte for både gamle og nye OpenCV versjoner
    try:
        # Ny OpenCV (4.7+)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    except AttributeError:
        # Gammel OpenCV
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    
    # Markør-oppsett for prosjektet
    markers = {
        0: "baseframe",
        1: "box_front", 
        2: "box_back",
        3: "box_left",
        4: "box_right"
    }
    
    # Størrelse på markørene
    marker_size_pixels = 200  # Størrelse på HELE markøren inkludert hvit kant
    
    # Lag mappe for markørene
    output_dir = "/home/jlaybakk/Gruppeprosjekt/Ar"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    print("Genererer ArUco-markører...")
    print(f"Markørstørrelse: {marker_size_pixels}x{marker_size_pixels} piksler")
    print("\nVIKTIG OM UTSKRIFT:")
    print("- HELE bildet (inkludert hvit kant) skal printes")
    print("- Mål størrelsen på HELE den utskrevne markøren")
    print("- Den hvite kanten rundt er VIKTIG for deteksjon!")
    print("-" * 50)
    
    # Generer hver markør
    for marker_id, name in markers.items():
        # Generer markør
        marker_image = np.zeros((marker_size_pixels, marker_size_pixels), dtype=np.uint8)
        try:
            # Ny OpenCV (4.7+)
            marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_pixels)
        except AttributeError:
            # Gammel OpenCV
            marker_image = aruco.drawMarker(aruco_dict, marker_id, marker_size_pixels, marker_image, 1)
        
        # Legg til ekstra hvit kant for bedre deteksjon
        border_size = 50
        marker_with_border = cv2.copyMakeBorder(
            marker_image,
            border_size, border_size, border_size, border_size,
            cv2.BORDER_CONSTANT,
            value=255
        )
        
        # Legg til tekst med ID og navn
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = f"ID: {marker_id} - {name.upper()}"
        text_size = cv2.getTextSize(text, font, 0.8, 2)[0]
        text_x = (marker_with_border.shape[1] - text_size[0]) // 2
        text_y = marker_with_border.shape[0] - 20
        
        cv2.putText(marker_with_border, text, (text_x, text_y), 
                   font, 0.8, (0, 0, 0), 2)
        
        # Lagre markør
        filename = f"{output_dir}/aruco_{marker_id}_{name}.png"
        cv2.imwrite(filename, marker_with_border)
        print(f"Generert: {filename} (ID: {marker_id})")
        
        # Vis preview
        cv2.imshow(f'ArUco {marker_id} - {name}', cv2.resize(marker_with_border, (300, 300)))
    
    print("\n" + "="*50)
    print("INSTRUKSJONER FOR UTSKRIFT:")
    print("1. Print alle 5 markørene på hvitt papir")
    print("2. VIKTIG: Behold den hvite kanten rundt markøren!")
    print("3. Mål størrelsen på HELE den utskrevne markøren (inkl. hvit kant)")
    print("4. Lim markørene på boksen og baseframe")
    print("5. Noter ned fysisk størrelse for bruk i deteksjonskoden")
    print("="*50)
    
    # Lag også et referanseark med alle markørene
    create_reference_sheet(markers, aruco_dict, output_dir)
    
    print("\nTrykk en tast for å lukke vinduene...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def create_reference_sheet(markers, aruco_dict, output_dir):
    """Lager et referanseark med alle markørene"""
    # A4-størrelse i 300 DPI
    a4_width = 2480  # 210mm
    a4_height = 3508  # 297mm
    
    sheet = np.ones((a4_height, a4_width), dtype=np.uint8) * 255
    
    # Markørstørrelse på arket
    marker_size = 400
    padding = 100
    
    # Tittel
    font = cv2.FONT_HERSHEY_SIMPLEX
    title = "ArUco Markers - Robot Box Project"
    title_size = cv2.getTextSize(title, font, 2, 3)[0]
    title_x = (a4_width - title_size[0]) // 2
    cv2.putText(sheet, title, (title_x, 150), font, 2, (0, 0, 0), 3)
    
    # Plasser markørene
    positions = [
        (padding, 400),  # Øverst venstre
        (a4_width - marker_size - padding, 400),  # Øverst høyre
        (padding, 1200),  # Midten venstre
        (a4_width - marker_size - padding, 1200),  # Midten høyre
        ((a4_width - marker_size) // 2, 2000)  # Nederst senter
    ]
    
    for i, (marker_id, name) in enumerate(markers.items()):
        if i < len(positions):
            x, y = positions[i]
            
            # Generer markør
            try:
                # Ny OpenCV (4.7+)
                marker = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
            except AttributeError:
                # Gammel OpenCV
                marker = aruco.drawMarker(aruco_dict, marker_id, marker_size)
            
            # Plasser på arket
            sheet[y:y+marker_size, x:x+marker_size] = marker
            
            # Legg til label
            label = f"ID: {marker_id} - {name.upper()}"
            label_y = y + marker_size + 50
            cv2.putText(sheet, label, (x, label_y), font, 1.2, (0, 0, 0), 2)
    
    # Instruksjoner nederst
    instructions = [
        "UTSKRIFTSINSTRUKSJONER:",
        "1. Print dette arket i 100% størrelse (ikke skaler)",
        "2. Klipp ut hver markør MED hvit kant",
        "3. Mål og noter størrelsen på markørene",
        "4. Den hvite kanten er VIKTIG for deteksjon!"
    ]
    
    y_pos = 2800
    for instruction in instructions:
        cv2.putText(sheet, instruction, (padding, y_pos), font, 1, (0, 0, 0), 2)
        y_pos += 60
    
    # Lagre referanseark
    reference_file = f"{output_dir}/aruco_reference_sheet.png"
    cv2.imwrite(reference_file, sheet)
    print(f"\nLagret referanseark: {reference_file}")
    
    # Vis preview
    preview = cv2.resize(sheet, (620, 877))  # A4 preview størrelse
    cv2.imshow('Reference Sheet', preview)

def visualize_marker_structure():
    """Visualiserer strukturen til en ArUco-markør"""
    print("\nArUco Markør Struktur:")
    print("="*40)
    print("En 4x4 ArUco-markør består av:")
    print("- 4x4 = 16 svarte/hvite ruter (bits)")
    print("- Svart kant rundt hele markøren")
    print("- Hvit kant utenfor (viktig for deteksjon)")
    print("\nEksempel 4x4 markør:")
    print("  ┌─────────────────┐")
    print("  │ ┌─┬─┬─┬─┬─┬─┐ │")  
    print("  │ ├─┼─┼─┼─┼─┼─┤ │")
    print("  │ │■│□│□│■│□│■│ │")
    print("  │ ├─┼─┼─┼─┼─┼─┤ │")
    print("  │ │□│■│■│□│■│□│ │")
    print("  │ ├─┼─┼─┼─┼─┼─┤ │")
    print("  │ │■│■│□│■│□│■│ │")
    print("  │ ├─┼─┼─┼─┼─┼─┤ │")
    print("  │ │□│□│■│□│■│□│ │")
    print("  │ ├─┼─┼─┼─┼─┼─┤ │")
    print("  │ └─┴─┴─┴─┴─┴─┘ │")
    print("  └─────────────────┘")
    print("\n■ = Svart, □ = Hvit")
    print("="*40)

if __name__ == "__main__":
    # Vis struktur
    visualize_marker_structure()
    
    # Generer markørene
    generate_aruco_markers()
    
    print("\nFerdig! Markørene er lagret i 'aruco_markers' mappen.")
    print("\nNeste steg:")
    print("1. Print markørene (helst på matt papir)")
    print("2. Mål størrelsen (f.eks. 5x5 cm)")
    print("3. Oppdater marker_size i deteksjonskoden")
    print("4. Lim på boks og baseframe")