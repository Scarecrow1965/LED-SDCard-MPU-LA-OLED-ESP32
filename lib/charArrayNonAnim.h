// +-------------------------------------------------------------
//
// Equipment:
// ESP32, OLED SSD1306
//
// File: charArrayNonAnim.h
//
// Description:
//
// Creates graphical displays on the SSD1306 OLED Display of char
// arrays which holds non-animated graphics on the ESP 32 platform
//
// History:     15-Nov-2023     Scarecrow1965   Created
//
// +-------------------------------------------------------------

// install ibraries
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "seeOLED.h"

extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
extern uint8_t oled_LineH;
extern bool bLED;
extern Adafruit_SSD1306 display;

class charArrayNonAnim
{
private:
    /* data */

    const uint8_t totalNonAnimArray = 55; // ensure this is the same as the number of frames in the array below

    // ======================================================================================================
    // OLD STRUCT AND ARRAY WITH LONG FILENAME FOR ESP32 USE
    // ======================================================================================================
    // struct Frame
    // {
    //     const unsigned char *data;
    //     uint8_t width;
    //     uint8_t height;
    //     const String name;
    //     const int dataSize;
    //     const String filename;
    // };

    // Frame framesInfo[55] = {
    //     {adhd_Logo, height_width, height_width, "ADHD Logo", 0, "adhd_Logo.bin"},                                                            //     framesInfo[0]. == {ADHD_Logo, 50, 50, "ADHD LOGO"},
    //     {arduino_Logo, 128, 64, "Arduino Logo", 0, "/arduino_Logo.bin"},                                                                     //     framesInfo[1]. == {arduino_Logo, 128, 64, "Arduino Logo"},
    //     {arduino_Logo2, height_width, height_width, "Arduino Logo", 0, "/arduino_Logo2.bin"},                                                //     framesInfo[2]. == {arduino_Logo2, 50, 50, "Arduino Logo"},
    //     {amd_Logo, height_width, height_width, "AMD Logo", 0, "/amd_Logo.bin"},                                                              //     framesInfo[3]. == {amd_Logo, 50, 50, "AMD Logo"},
    //     {autism_Logo, height_width, height_width, "Autism Logo", 0, "/autism_Logo.bin"},                                                     //     framesInfo[4]. == {autism_Logo, 50, 50, "Autism Logo"},
    //     {c_plus_plus_Logo, height_width, height_width, "C++ Logo", 0, "/c_plus_plus_Logo.bin"},                                              //     framesInfo[5]. == {c_plus_plus_Icon, 50, 50, "C++ ICON"},
    //     {codepen_Logo, height_width, height_width, "CodePen Logo", 0, "/codepen_Logo.bin"},                                                  //     framesInfo[6]. == {codepen_Icon, 50, 50, "CodePen ICON"},
    //     {css3_Logo, height_width, height_width, "CSS 3 Logo", 0, "/css3_Logo.bin"},                                                          //     framesInfo[7]. == {css3_Icon, 50, 50, "CSS 3 ICON"},
    //     {docker_Logo, height_width, height_width, "Docker Logo", 0, "/docker_Logo.bin"},                                                     //     framesInfo[8]. == {docker_Icon, 50, 50, "Docker ICON"},
    //     {git_Logo, height_width, height_width, "GIT Logo", 0, "/git_Logo.bin"},                                                              //     framesInfo[9]. == {git_Icon, 50, 50, "GIT ICON"},
    //     {github_Logo, height_width, height_width, "GitHub Logo", 0, "/github_Logo.bin"},                                                     //     framesInfo[10]. == {github_Icon, 50, 50, "GitHub ICON"},
    //     {html5_Logo, height_width, height_width, "HTML 5 Logo", 0, "/html5_Logo.bin"},                                                       //     framesInfo[11]. == {html5_Icon, 50, 50, "HTML 5 ICON"},
    //     {java_Logo, height_width, height_width, "Java Logo", 0, "/java_Logo.bin"},                                                           //     framesInfo[12]. == {java_Icon, 50, 50, "Java ICON"},
    //     {javascript_Logo, height_width, height_width, "JavaScript Logo", 0, "/javascript_Logo.bin"},                                         //     framesInfo[13]. == {javascript_Icon, 50, 50, "JavaScript ICON"},
    //     {jenkins_Logo, height_width, height_width, "Jenkins Logo", 0, "/jenkins_Logo.bin"},                                                  //     framesInfo[14]. == {jenkins_Icon, 50, 50, "Jenkins ICON"},
    //     {jquery_Logo, height_width, height_width, "JQuery Logo", 0, "/jquery_Logo.bin"},                                                     //     framesInfo[15]. == {jquery_Icon, 50, 50, "JQuery ICON"},
    //     {kubernetes_Logo, height_width, height_width, "Kubernetes Logo", 0, "/kubernetes_Logo.bin"},                                         //     framesInfo[16]. == {kubernetes_Icon, 50, 50, "Kubernetes ICON"},
    //     {mysql_Logo, height_width, height_width, "MySQL Logo", 0, "/mysql_Logo.bin"},                                                        //     framesInfo[17]. == {mysql_Logo, 50, 50, "MySQL Logo"},
    //     {php_Logo, height_width, height_width, "php Logo", 0, "/php_Logo.bin"},                                                              //     framesInfo[18]. == {php_Logo, 50, 50, "php Logo"},
    //     {pinterest_Logo, height_width, height_width, "Pinterest Logo", 0, "/pinterest_Logo.bin"},                                            //     framesInfo[19]. == {pinterest, 50, 50, "Pinterest"},
    //     {python_Logo, height_width, height_width, "Python Logo", 0, "/python_Logo.bin"},                                                     //     framesInfo[20]. == {python_Icon, 50, 50, "Python ICON"},
    //     {raspberry_pi_Logo, height_width, height_width, "Raspberry Pi Logo", 0, "/"},                                                        //     framesInfo[21]. == {raspberry_pi_Logo, 50, 50, "Raspberry Pi Logo"},
    //     {ruby_programming_language_Logo, height_width, height_width, "Ruby Programming Language", 0, "/ruby_programming_language_Logo.bin"}, //     framesInfo[22]. == {ruby_programming_language, 50, 50, "Ruby Programming Language"},
    //     {united_nations_Logo, height_width, height_width, "United Nations Logo", 0, "/united_nations_Logo.bin"},                             //     framesInfo[23]. == {united_nations_Icon, 50, 50, "United Nations Icon"},
    //     {air_Drone_Icon, height_width, height_width, "Air Drone ICON", 0, "/air_Drone_Icon.bin"},                                            //     framesInfo[24]. == {air_Drone, 50, 50, "Air Drone"},
    //     {aircraft_Mechanic_Icon, height_width, height_width, "Aircraft Mechanic ICON", 0, "/aircraft_Mechanic_Icon.bin"},                    //     framesInfo[25]. == {aircraft_Mechanic, 50, 50, "Aircraft Mechanic"},
    //     {transport_airplane_Icon, height_width, height_width, "Transport Aircraft ICON", 0, "/transport_airplane_Icon.bin"},                 //     framesInfo[26]. == {transport_airplane, 50, 50, "Transport Aircraft"},
    //     {fighter_airplane1_Icon, height_width, height_width, "Fighter Aircraft #1 ICON", 0, "/fighter_airplane1_Icon.bin"},                  //     framesInfo[27]. == {fighter_airplane1, 50, 50, "Fighter Aircraft #1"},
    //     {fighter_airplane2_Icon, height_width, height_width, "Fighter Aircraft #2 ICON", 0, "/fighter_airplane2_Icon.bin"},                  //     framesInfo[28]. == {fighter_airplane2, 50, 50, "Fighter Aircraft #2"},
    //     {training_airplane_Icon, height_width, height_width, "Training Aircraft ICON", 0, "/training_airplane_Icon.bin"},                    //     framesInfo[29]. == {training_airplane, 50, 50, "Training Aircraft"},
    //     {airplane_landing_Icon, height_width, height_width, "Airplane Landing ICON", 0, "/airplane_landing_Icon.bin"},                       //     framesInfo[30]. == {airplane_landing, 50, 50, "Airplane Landing ICON"},
    //     {helicopter1_Icon, height_width, height_width, "Helicopter #1 ICON", 0, "/helicopter1_Icon.bin"},                                    //     framesInfo[31]. == {helicopter1, 50, 50, "Helicopter #1"},
    //     {helicopter2_Icon, height_width, height_width, "Helicopter #2 ICON", 0, "/helicopter2_Icon.bin"},                                    //     framesInfo[32]. == {helicopter2, 50, 50, "Helicopter #2"},
    //     {armament_Icon, height_width, height_width, "Armament ICON", 0, "/armament_Icon.bin"},                                               //     framesInfo[33]. == {armament, 50, 50, "Armament"},
    //     {navigation_compass_Icon, height_width, height_width, "Navigation Compass ICON", 0, "/navigation_compass_Icon.bin"},                 //     framesInfo[34]. == {navigation_compass, 50, 50, "Navigation Compass"},
    //     {radar_Icon, height_width, height_width, "Radar ICON", 0, "/radar_Icon.bin"},                                                        //     framesInfo[35]. == {radar, 50, 50, "Radar"},
    //     {radio_waves_Icon, height_width, height_width, "Radio Waves ICON", 0, "/radio_waves_Icon.bin"},                                      //     framesInfo[36]. == {radio_waves, 50, 50, "Radio Waves"},
    //     {sonar_Icon, height_width, height_width, "Sonar ICON", 0, "/sonar_Icon.bin"},                                                        //     framesInfo[37]. == {sonar, 50, 50, "Sonar"},
    //     {address_Location_Icon, height_width, height_width, "Address Location ICON", 0, "/address_Location_Icon.bin"},                       //     framesInfo[38]. == {address_Location, 50, 50, "Address Location"},
    //     {bookmark_Icon, height_width, height_width, "Bookmark ICON", 0, "/bookmark_Icon.bin "},                                              //     framesInfo[39]. == {bookmark_Icon, 50, 50, "Bookmark ICON"},
    //     {checkmark_Icon, height_width, height_width, "Checkmark OK ICON", 0, "/checkmark_Icon.bin"},                                         //     framesInfo[40]. == {checkmark_OK_Icon, 50, 50, "Checkmark OK ICON"},
    //     {close_Icon, height_width, height_width, "Close ICON", 0, "/close_Icon.bin"},                                                        //     framesInfo[41]. == {close_Icon, 50, 50, "Close ICON"},
    //     {folder_Icon, height_width, height_width, "Folder ICON", 0, "/folder_Icon.bin"},                                                     //     framesInfo[42]. == {folder_Icon, 50, 50, "Folder ICON"},
    //     {home_Icon, height_width, height_width, "Home ICON", 0, "/home_Icon.bin"},                                                           //     framesInfo[43]. == {home_Icon, 50, 50, "Home ICON"},
    //     {idea_Icon, height_width, height_width, "Idea ICON", 0, "/idea_Icon.bin"},                                                           //     framesInfo[44]. == {idea_Icon, 50, 50, "Idea ICON"},
    //     {listen_Icon, height_width, height_width, "Listen ICON", 0, "/listen_Icon.bin"},                                                     //     framesInfo[45]. == {listen_Icon, 50, 50, "Listen ICON"},
    //     {menu_Icon, height_width, height_width, "Menu ICON", 0, "/menu_Icon.bin"},                                                           //     framesInfo[46]. == {menu_Icon, 50, 50, "Menu ICON"},
    //     {services_Icon, height_width, height_width, "Services ICON", 0, "/services_Icon.bin"},                                               //     framesInfo[47]. == {services_Icon, 50, 50, "Services ICON"},
    //     {settings_Icon, height_width, height_width, "Settings ICON", 0, "/settings_Icon.bin"},                                               //     framesInfo[48]. == {settings_Icon, 50, 50, "Settings ICON"},
    //     {shield_Icon, height_width, height_width, "Shield ICON", 0, "/shield_Icon.bin"},                                                     //     framesInfo[49]. == {shield_Icon, 50, 50, "Shield ICON"},
    //     {sound_bars_Icon, height_width, height_width, "Sound Bars ICON", 0, "/sound_bars_Icon.bin"},                                         //     framesInfo[50]. == {sound_bars, 50, 50, "Sound Bars"},
    //     {speech_bubble_Icon, height_width, height_width, "Speech Bubble ICON", 0, "/speech_bubble_Icon.bin"},                                //     framesInfo[51]. == {speech_bubble, 50, 50, "Speech Bubble"},
    //     {unavailable_Icon, height_width, height_width, "Unavailable ICON", 0, "/unavailable_Icon.bin"},                                      //     framesInfo[52]. == {unavailable_Icon, 50, 50, "Unavailable Icon"},
    //     {ufo_Icon, height_width, height_width, "UFO ICON", 0, "/ufo_Icon.bin"},                                                              //     framesInfo[53]. == {ufo, 50, 50, "UFO"},
    //     {sub_red_october_Icon, height_width, height_width, "Red October Submarine ICON", 0, "/sub_red_october_Icon.bin"},                    //     framesInfo[54]. == {sub_red_october, 50, 50, "Red October Submarine"},
    // };
    // ======================================================================================================

    struct Icon
    {
        const String name;
        uint8_t width;
        uint8_t height;
        const char *fileName;
    };

    // Array with short names for Arduino code
    Icon charArrayNonAnimation[55] {
        {"ADHD Logo", height_width2, height_width2, "/ch_adhdL.bin"},                  //     charNonAnim[0]. == {"ADHD LOGO", 50, 50, "/ch_adhdL.bin"},
        {"Arduino Logo", 128, 64, "/ch_ardL.bin"},                                   //     charNonAnim[1]. == {"Arduino Logo", 128, 64, "/ch_ardL.bin"},
        {"Arduino Logo2", height_width2, height_width2, "/ch_ard2L.bin"},              //     charNonAnim[2]. == {"Arduino Logo2", 50, 50, "/ch_ard2L.bin"},
        {"AMD Logo", height_width2, height_width2, "/ch_amdL.bin"},                    //     charNonAnim[3]. == {"AMD Logo", 50, 50, "/ch_amdL.bin"},
        {"Autism Logo", height_width2, height_width2, "/ch_autL.bin"},                 //     charNonAnim[4]. == {"Autism Logo", 50, 50, "/ch_autL.bin"},
        {"C++ Logo", height_width2, height_width2, "/ch_cppL.bin"},                    //     charNonAnim[5]. == {"C++ Logo", 50, 50, "/ch_cppL.bin"},
        {"CodePen Logo", height_width2, height_width2, "/ch_cdpnL.bin"},               //     charNonAnim[6]. == {"CodePen Logo", 50, 50, "/ch_cdpnL.bin"},
        {"CSS 3 Logo", height_width2, height_width2, "/ch_css3L.bin"},                 //     charNonAnim[7]. == {"CSS 3 Logo", 50, 50, "/ch_css3L.bin"},
        {"Docker Logo", height_width2, height_width2, "/ch_dockL.bin"},                //     charNonAnim[8]. == {"Docker Logo", 50, 50, "/ch_dockL.bin"},
        {"GIT Logo", height_width2, height_width2, "/ch_git_L.bin"},                   //     charNonAnim[9]. == {"GIT Logo", 50, 50, "/ch_git_L.bin"},
        {"GitHub Logo", height_width2, height_width2, "/ch_gthbL.bin"},                //     charNonAnim[10]. == {"GitHub Logo", 50, 50, "/ch_gthbL.bin"},
        {"HTML 5 Logo", height_width2, height_width2, "/ch_htmlL.bin"},                //     charNonAnim[11]. == {"HTML 5 Logo", 50, 50, "/ch_htmlL.bin"},
        {"Java Logo", height_width2, height_width2, "/ch_javaL.bin"},                  //     charNonAnim[12]. == {"Java Logo", 50, 50, "/ch_javaL.bin"},
        {"JavaScript Logo", height_width2, height_width2, "/ch_js_L.bin"},             //     charNonAnim[13]. == {"JavaScript Logo", 50, 50, "/ch_js_L.bin"},
        {"Jenkins Logo", height_width2, height_width2, "/ch_jnksL.bin"},               //     charNonAnim[14]. == {"Jenkins Logo", 50, 50, "/ch_jnksL.bin"},
        {"JQuery Logo", height_width2, height_width2, "/ch_jquyL.bin"},                //     charNonAnim[15]. == {"JQuery Logo", 50, 50, "/ch_jquyL.bin"},
        {"Kubernetes Logo", height_width2, height_width2, "/ch_kubsL.bin"},            //     charNonAnim[16]. == {"Kubernetes Logo", 50, 50, "/ch_kubsL.bin"},
        {"MySQL Logo", height_width2, height_width2, "/ch_mysqL.bin"},                 //     charNonAnim[17]. == {"MySQL Logo", 50, 50, "/ch_mysqL.bin"},
        {"php Logo", height_width2, height_width2, "/ch_php_L.bin"},                   //     charNonAnim[18]. == {"php Logo", 50, 50, "/ch_php_L.bin"},
        {"Pinterest Logo", height_width2, height_width2, "/ch_prstL.bin"},             //     charNonAnim[19]. == {"Pinterest Logo", 50, 50, "/ch_prstL.bin"},
        {"Python Logo", height_width2, height_width2, "/ch_pyt_L.bin"},                //     charNonAnim[20]. == {"Python Logo", 50, 50, "/ch_pyt_L.bin"},
        {"Raspberry Pi Logo", height_width2, height_width2, "/ch_rspiL.bin"},          //     charNonAnim[21]. == {"Raspberry Pi Logo", 50, 50, "/ch_rspiL.bin"},
        {"Ruby Programming Language", height_width2, height_width2, "/ch_rubyL.bin"},  //     charNonAnim[22]. == {"Ruby Programming Language", 50, 50, "/ch_rubyL.bin"},
        {"United Nations Logo", height_width2, height_width2, "/ch_un_L.bin"},         //     charNonAnim[23]. == {"United Nations Logo", 50, 50, "/ch_un_L.bin"},
        {"Air Drone ICON", height_width2, height_width2, "/ch_airDi.bin"},             //     charNonAnim[24]. == {"Air Drone ICON", 50, 50, "/ch_airDi.bin"},
        {"Aircraft Mechanic ICON", height_width2, height_width2, "/ch_acMci.bin"},     //     charNonAnim[25]. == {"Aircraft Mechanic ICON", 50, 50, "/ch_acMci.bin"},
        {"Transport Aircraft ICON", height_width2, height_width2, "/ch_tpt_i.bin"},    //     charNonAnim[26]. == {"Transport Aircraft ICON", 50, 50, "/ch_tpt_i.bin"},
        {"Fighter Aircraft #1 ICON", height_width2, height_width2, "/ch_fgt_i.bin"},   //     charNonAnim[27]. == {"Fighter Aircraft #1 ICON", 50, 50, "/ch_fgt_i.bin"},
        {"Fighter Aircraft #2 ICON", height_width2, height_width2, "/ch_fgt2i.bin"},   //     charNonAnim[28]. == {"Fighter Aircraft #2 ICON", 50, 50, "/ch_fgt2i.bin"},
        {"Training Aircraft ICON", height_width2, height_width2, "/ch_trg_i.bin"},     //     charNonAnim[29]. == {"Training Aircraft ICON", 50, 50, "/ch_trg_i.bin"},
        {"Airplane Landing ICON", height_width2, height_width2, "/ch_acldgi.bin"},     //     charNonAnim[30]. == {"Airplane Landing ICON", 50, 50, "/ch_acldgi.bin"},
        {"Helicopter #1 ICON", height_width2, height_width2, "/ch_heli.bin"},          //     charNonAnim[31]. == {"Helicopter #1 ICON", 50, 50, "/ch_heli.bin"},
        {"Helicopter #2 ICON", height_width2, height_width2, "/ch_hel2i.bin"},         //     charNonAnim[32]. == {"Helicopter #2 ICON", 50, 50, "/ch_hel2i.bin"},
        {"Armament ICON", height_width2, height_width2, "/ch_arm_i.bin"},              //     charNonAnim[33]. == {"Armament ICON", 50, 50, "/ch_arm_i.bin"},
        {"Navigation Compass ICON", height_width2, height_width2, "/ch_nav_i.bin"},    //     charNonAnim[34]. == {"Navigation Compass ICON", 50, 50, "/ch_nav_i.bin"},
        {"Radar ICON", height_width2, height_width2, "/ch_rdr_i.bin"},                 //     charNonAnim[35]. == {"Radar ICON", 50, 50, "/ch_rdr_i.bin"},
        {"Radio Waves ICON", height_width2, height_width2, "/ch_rad_i.bin"},           //     charNonAnim[36]. == {"Radio Waves ICON", 50, 50, "/ch_rad_i.bin"},
        {"Sonar ICON", height_width2, height_width2, "/ch_snr_i.bin"},                 //     charNonAnim[37]. == {"Sonar ICON", 50, 50, "/ch_snr_i.bin"},
        {"Address Location ICON", height_width2, height_width2, "/ch_adLoi.bin"},      //     charNonAnim[38]. == {"Address Location ICON", 50, 50, "/ch_adLoi.bin"},
        {"Bookmark ICON", height_width2, height_width2, "/ch_booki.bin"},              //     charNonAnim[39]. == {"Bookmark ICON", 50, 50, "/ch_booki.bin"},
        {"Checkmark OK ICON", height_width2, height_width2, "/ch_chk_i.bin"},          //     charNonAnim[40]. == {"Checkmark OK ICON", 50, 50, "/ch_chk_i.bin"},
        {"Close ICON", height_width2, height_width2, "/ch_cls_i.bin"},                 //     charNonAnim[41]. == {"Close ICON", 50, 50, "/ch_cls_i.bin"},
        {"Folder ICON", height_width2, height_width2, "/ch_fld_i.bin"},                //     charNonAnim[42]. == {"Folder ICON", 50, 50, "/ch_fld_i.bin"},
        {"Home ICON", height_width2, height_width2, "/ch_homei.bin"},                  //     charNonAnim[43]. == {"Home ICON", 50, 50, "/ch_homei.bin"},
        {"Idea ICON", height_width2, height_width2, "/ch_ideai.bin"},                  //     charNonAnim[44]. == {"Idea ICON", 50, 50, "/ch_ideai.bin"},
        {"Listen ICON", height_width2, height_width2, "/ch_lst_i.bin"},                //     charNonAnim[45]. == {"Listen ICON", 50, 50, "/ch_lst_i.bin"},
        {"Menu ICON", height_width2, height_width2, "/ch_menui.bin"},                  //     charNonAnim[46]. == {"Menu ICON", 50, 50, "/ch_menui.bin"},
        {"Services ICON", height_width2, height_width2, "/ch_svs_i.bin"},              //     charNonAnim[47]. == {"Services ICON", 50, 50, "/ch_svs_i.bin"},
        {"Settings ICON", height_width2, height_width2, "/ch_set_i.bin"},              //     charNonAnim[48]. == {"Settings ICON", 50, 50, "/ch_set_i.bin"},
        {"Shield ICON", height_width2, height_width2, "/ch_shldi.bin"},                //     charNonAnim[49]. == {"Shield ICON", 50, 50, "/ch_shldi.bin"},
        {"Sound Bars ICON", height_width2, height_width2, "/ch_snd_i.bin"},            //     charNonAnim[50]. == {"Sound Bars ICON", 50, 50, "/ch_snd_i.bin"},
        {"Speech Bubble ICON", height_width2, height_width2, "/ch_spb_i.bin"},         //     charNonAnim[51]. == {"Speech Bubble ICON", 50, 50, "/ch_spb_i.bin"},
        {"Unavailable ICON", height_width2, height_width2, "/ch_unavi.bin"},           //     charNonAnim[52]. == {"Unavailable ICON", 50, 50, "/ch_unavi.bin"},
        {"UFO ICON", height_width2, height_width2, "/ch_ufo_i.bin"},                   //     charNonAnim[53]. == {"UFO ICON", 50, 50, "/ch_ufo_i.bin"},
        {"Red October Submarine ICON", height_width2, height_width2, "/ch_sub_i.bin"}, //     charNonAnim[54]. == {"Red October Submarine ICON", 50, 50, "/ch_sub_i.bin"},
    };

    uint8_t *charGraphicsBuffer;
    size_t charGraphicsSize;
    // end private data

public:
    // charArrayNonAnim(/* args */);
    charArrayNonAnim() // Constructor function but not in class
    {
        Serial.println("Char Array Non-Anim Constructor started");
    };

    // Destructor
    ~charArrayNonAnim()
    {
        Serial.println("Char Array Non-Anim completed");
        display.clearDisplay();
        u8g2.clearBuffer();
    };

    // char array non_Anim function to run through individual icons
    void charArrayNon_Anim(int i) // Constructor function but not in class
    {
        //     Serial.println("Char Array Non-Anim Constructor started"); // used for testing purposes only
        display.clearDisplay();

        loadBinFile(charArrayNonAnimation[i].fileName, &charGraphicsBuffer, &charGraphicsSize);

        display.drawBitmap(0, 16, charGraphicsBuffer, charArrayNonAnimation[i].width, charArrayNonAnimation[i].height, 1);
        display.display();
        delete[] charGraphicsBuffer;
    }

    // LOGOS FIRST
    void ADHDLogo()
    {
        //     charNonAnim[0]. == {"ADHD LOGO", 50, 50, "/ch_adhdL.bin"},
        charArrayNon_Anim(0);
    }; // end to calling the ADHD Logo function ONLY

    void ARDUINOLogo()
    {
        //     charNonAnim[1]. == {"Arduino Logo", 128, 64, "/ch_ardL.bin"},
        charArrayNon_Anim(1);
    }; // end to calling the Arduino Logo function ONLY

    void ARDUINOLogo2()
    {
        //     charNonAnim[2]. == {"Arduino Logo2", 50, 50, "/ch_ard2L.bin"},
        charArrayNon_Anim(2);
    }; // end to calling the Arduino Logo function ONLY

    void AMDLogo()
    {
        //     charNonAnim[3]. == {"AMD Logo", 50, 50, "/ch_amdL.bin"},
        charArrayNon_Anim(3);
    }; // end to calling the AMD Logo function ONLY;

    void AUTISMLogo()
    {
        //     charNonAnim[4]. == {"Autism Logo", 50, 50, "/ch_autL.bin"},
        charArrayNon_Anim(4);
    }; // end to calling the Autism Logo function ONLY

    void CPLUSPLUSLogo()
    {
        //     charNonAnim[5]. == {"C++ Logo", 50, 50, "/ch_cppL.bin"},
        charArrayNon_Anim(5);
    }; // end to calling the C++ Logo function ONLY

    void CODEPENLogo()
    {
        //     charNonAnim[6]. == {"CodePen Logo", 50, 50, "/ch_cdpnL.bin"},
        charArrayNon_Anim(6);
    }; // end to calling the CodePen.io Logo function ONLY

    void CSS3Logo()
    {
        //     charNonAnim[7]. == {"CSS 3 Logo", 50, 50, "/ch_css3L.bin"},
        charArrayNon_Anim(7);
    }; // end to calling the CSS 3 Logo function ONLY

    void DOCKERLogo()
    {
        //     charNonAnim[8]. == {"Docker Logo", 50, 50, "/ch_dockL.bin"},
        charArrayNon_Anim(8);
    }; // end to calling the Docker Logo function ONLY

    void GITLogo()
    {
        //     charNonAnim[9]. == {"GIT Logo", 50, 50, "/ch_git_L.bin"},
        charArrayNon_Anim(9);
    }; // end to calling the Git Logo function ONLY

    void GITHUBLogo()
    {
        //     charNonAnim[10]. == {"GitHub Logo", 50, 50, "/ch_gthbL.bin"},
        charArrayNon_Anim(10);
    }; // end to calling the GitHub Logo function ONLY

    void HTML5Logo()
    {
        //     charNonAnim[11]. == {"HTML 5 Logo", 50, 50, "/ch_htmlL.bin"},
        charArrayNon_Anim(11);
    }; // end to calling the HTML 5 Logo function ONLY

    void JAVALogo()
    {
        //     charNonAnim[12]. == {"Java Logo", 50, 50, "/ch_javaL.bin"},
        charArrayNon_Anim(12);
    }; // end to calling the Java Logo function ONLY

    void JAVASCRIPTLogo()
    {
        //     charNonAnim[13]. == {"JavaScript Logo", 50, 50, "/ch_js_L.bin"},
        charArrayNon_Anim(13);
    }; // end to calling the JavaScript Logo function ONLY

    void JENKINSLogo()
    {
        //     charNonAnim[14]. == {"Jenkins Logo", 50, 50, "/ch_jnksL.bin"},
        charArrayNon_Anim(14);
    }; // end to calling the Jenkins Logo function ONLY

    void JQUERYLogo()
    {
        //     charNonAnim[15]. == {"JQuery Logo", 50, 50, "/ch_jquyL.bin"},
        charArrayNon_Anim(15);
    }; // end to calling the JQuery Logo function ONLY

    void KUBERNETESLogo()
    {
        //     charNonAnim[16]. == {"Kubernetes Logo", 50, 50, "/ch_kubsL.bin"},
        charArrayNon_Anim(16);
    }; // end to calling the Kubernetes Logo function ONLY

    void MYSQLLogo()
    {
        //     charNonAnim[17]. == {"MySQL Logo", 50, 50, "/ch_mysqL.bin"},
        charArrayNon_Anim(17);
    }; // end to calling the MySQL Logo function ONLY

    void PHPLogo()
    {
        //     charNonAnim[18]. == {"php Logo", 50, 50, "/ch_php_L.bin"},
        charArrayNon_Anim(18);
    }; // end to calling the php Logo function ONLY

    void PINTERESTLogo()
    {
        //     charNonAnim[19]. == {"Pinterest Logo", 50, 50, "/ch_prstL.bin"},
        charArrayNon_Anim(19);
    }; // end to calling the PInterest Logo function ONLY

    void PYTHONLogo()
    {
        //     charNonAnim[20]. == {"Python Logo", 50, 50, "/ch_pyt_L.bin"},
        charArrayNon_Anim(20);
    }; // end to calling the Python Logo function ONLY

    void RASPBERRYPILogo()
    {
        //     charNonAnim[21]. == {"Raspberry Pi Logo", 50, 50, "/ch_rspiL.bin"},
        charArrayNon_Anim(21);
    }; // end to calling the RaspberryPi Logo function ONLY

    void RUBYLogo()
    {
        //     charNonAnim[22]. == {"Ruby Programming Language", 50, 50, "/ch_rubyL.bin"},
        charArrayNon_Anim(22);
    }; // end to calling the Ruby Programming Logo function ONLY

    void UNITEDNATIONSLogo()
    {
        //     charNonAnim[23]. == {"United Nations Logo", 50, 50, "/ch_un_L.bin"},
        charArrayNon_Anim(23);
    }; // end to calling the United Nations Logo function ONLY

    // AIRCRAFT RELATED NEXT
    void droneIcon()
    {
        //     charNonAnim[24]. == {"Air Drone ICON", 50, 50, "/ch_airDi.bin"},
        charArrayNon_Anim(24);
    }; // end to calling the drone icon function ONLY

    void aircraftMechanicIcon()
    {
        //     charNonAnim[25]. == {"Aircraft Mechanic ICON", 50, 50, "/ch_acMci.bin"},
        charArrayNon_Anim(25);
    }; // end to calling the aircraft mechanic icon function ONLY

    void transportaircraftIcon()
    {
        //     charNonAnim[26]. == {"Transport Aircraft ICON", 50, 50, "/ch_tpt_i.bin"},
        charArrayNon_Anim(26);
    }; // end to calling the transport aircairplane landing/airport icon function ONLY

    void fighteraircraft1Icon()
    {
        //     charNonAnim[27]. == {"Fighter Aircraft #1 ICON", 50, 50, "/ch_fgt_i.bin"},
        charArrayNon_Anim(27);
    }; // end to calling the first fighter aircraft icon function ONLY

    void fighteraircraft2Icon()
    {
        //     charNonAnim[28]. == {"Fighter Aircraft #2 ICON", 50, 50, "/ch_fgt2i.bin"},
        charArrayNon_Anim(28);
    }; // end to calling the second fighter aircraft icon function ONLY

    void TrainingaircraftIcon()
    {
        //     charNonAnim[29]. == {"Training Aircraft ICON", 50, 50, "/ch_trg_i.bin"},
        charArrayNon_Anim(29);
    }; // end to calling the single rotary engine/training aircrasft icon function ONLY

    void AirplaneLandingIcon()
    {
        //     charNonAnim[30]. == {"Airplane Landing ICON", 50, 50, "/ch_acldgi.bin"},
        charArrayNon_Anim(30);
    }; // end to calling the airplane landing/airport icon function ONLY

    void helicopterIcon1()
    {
        //     charNonAnim[31]. == {"Helicopter #1 ICON", 50, 50, "/ch_heli.bin"},
        charArrayNon_Anim(31);
    }; // end to calling the helicopter icon function ONLY

    void helicopterIcon2()
    {
        //     charNonAnim[32]. == {"Helicopter #2 ICON", 50, 50, "/ch_hel2i.bin"},
        charArrayNon_Anim(32);
    }; // end to calling the helicopter icon function ONLY

    void armamentIcon()
    {
        //     charNonAnim[33]. == {"Armament ICON", 50, 50, "/ch_arm_i.bin"},
        charArrayNon_Anim(33);
    }; // end to calling the armament/rocket icon function ONLY

    void navigationCompassIcon()
    {
        //     charNonAnim[34]. == {"Navigation Compass ICON", 50, 50, "/ch_nav_i.bin"},
        charArrayNon_Anim(34);
    }; // end to calling the navigation/compass icon function ONLY

    void radarIcon()
    {
        //     charNonAnim[35]. == {"Radar ICON", 50, 50, "/ch_rdr_i.bin"},
        charArrayNon_Anim(35);
    }; // end to calling the black and white radar screen icon function ONLY

    void radiowavesIcon()
    {
        //     charNonAnim[36]. == {"Radio Waves ICON", 50, 50, "/ch_rad_i.bin"},
        charArrayNon_Anim(36);
    }; // end to calling the Radio Waves icon function ONLY

    void sonarIcon()
    {
        //     charNonAnim[37]. == {"Sonar ICON", 50, 50, "/ch_snr_i.bin"},
        charArrayNon_Anim(37);
    }; // end to calling the sonar screen icon function ONLY

    // OS RELATED NEXT
    void addressLocationIcon()
    {
        //     charNonAnim[38]. == {"Address Location ICON", 50, 50, "/ch_adLoi.bin"},
        charArrayNon_Anim(38);
    }; // end to calling the address loaction icon function ONLY

    void bookmarkIcon()
    {
        //     charNonAnim[39]. == {"Bookmark ICON", 50, 50, "/ch_booki.bin"},
        charArrayNon_Anim(39);
    }; // end to calling the bookmark icon function ONLY

    void CheckmarkOKIcon()
    {
        //     charNonAnim[40]. == {"Checkmark OK ICON", 50, 50, "/ch_chk_i.bin"},
        charArrayNon_Anim(40);
    }; // end to calling the checkmark OK icon function ONLY

    void closeIcon()
    {
        //     charNonAnim[41]. == {"Close ICON", 50, 50, "/ch_cls_i.bin"},
        charArrayNon_Anim(41);
    }; // end to calling the close icon function ONLY

    void folderIcon()
    {
        //     charNonAnim[42]. == {"Folder ICON", 50, 50, "/ch_fld_i.bin"},
        charArrayNon_Anim(42);
    }; // end to calling the folder icon function ONLY

    void homeIcon()
    {
        //     charNonAnim[43]. == {"Home ICON", 50, 50, "/ch_homei.bin"},
        charArrayNon_Anim(43);
    }; // end to calling the Home icon function ONLY

    void ideaIcon()
    {
        //     charNonAnim[44]. == {"Idea ICON", 50, 50, "/ch_ideai.bin"},
        charArrayNon_Anim(44);
    }; // end to calling the idea icon function ONLY

    void listenIcon()
    {
        //     charNonAnim[45]. == {"Listen ICON", 50, 50, "/ch_lst_i.bin"},
        charArrayNon_Anim(45);
    }; // end to calling the listening icon function ONLY

    void menuIcon()
    {
        //     charNonAnim[46]. == {"Menu ICON", 50, 50, "/ch_menui.bin"},
        charArrayNon_Anim(46);
    }; // end to calling the menu/hamburger icon function ONLY

    void servicesIcon()
    {
        //     charNonAnim[47]. == {"Services ICON", 50, 50, "/ch_svs_i.bin"},
        charArrayNon_Anim(47);
    }; // end to calling the services/gears icon function ONLY

    void settingsIcon()
    {
        //     charNonAnim[48]. == {"Settings ICON", 50, 50, "/ch_set_i.bin"},
        charArrayNon_Anim(48);
    }; // end to calling the settings/gear icon function ONLY

    void shieldIcon()
    {
        //     charNonAnim[49]. == {"Shield ICON", 50, 50, "/ch_shldi.bin"},
        charArrayNon_Anim(49);
    }; // end to calling the shield icon Logo function ONLY

    void sounbbarsIcon()
    {
        //     charNonAnim[50]. == {"Sound Bars ICON", 50, 50, "/ch_snd_i.bin"},
        charArrayNon_Anim(50);
    }; // end to calling the sound bars icon function ONLY

    void speechbubbleIcon()
    {
        //     charNonAnim[51]. == {"Speech Bubble ICON", 50, 50, "/ch_spb_i.bin"},
        charArrayNon_Anim(51);
    }; // end to calling the Speech Bubble icon function ONLY

    void unavailableIcon()
    {
        //     charNonAnim[52]. == {"Unavailable ICON", 50, 50, "/ch_unavi.bin"},
        charArrayNon_Anim(52);
    }; // end to calling the unavailable/don't go there icon function ONLY

    void ufoIcon()
    {
        //     charNonAnim[53]. == {"UFO ICON", 50, 50, "/ch_ufo_i.bin"},
        charArrayNon_Anim(53);
    }; // end to calling the UFO icon function ONLY

    void redoctobersubIcon()
    {
        //     charNonAnim[54]. == {"Red October Submarine ICON", 50, 50, "/ch_sub_i.bin"},
        charArrayNon_Anim(54);
    }; // end to calling the red october sub icon function ONLY

    void smallFramechar(uint8_t *charGraphicsBuffer)
    {
        // Serial.print("height_width = ");              // used for testing purposes only
        // Serial.println(height_width);                 // used for testing purposes only
        // Serial.print("charGraphicsBuffer = ");        // used for testing purposes only
        // for (size_t i = 0; i < charGraphicsSize; i++) // used for testing purposes only
        // {                                             // used for testing purposes only
        //     Serial.print(charGraphicsBuffer[i], HEX); // used for testing purposes only
        //     Serial.print(" ");                        // used for testing purposes only
        // }                                             // used for testing purposes only
        // Serial.println();                             // used for testing purposes only

        display.drawBitmap(0, 16, charGraphicsBuffer, height_width2, height_width2, 1);
        display.display();
    }; // end to display small framed graphics at the location just below the colour line

    // function to run through all the char array non-animated graphics
    void charArray_NonAnim()
    {
        // const int MAX_SIZE = 3072;
        for (uint8_t i = 0; i < totalNonAnimArray; i++)
        {
            loadBinFile(charArrayNonAnimation[i].fileName, &charGraphicsBuffer, &charGraphicsSize);

            int effectime = 20;
            display.clearDisplay();

            while (effectime > 0)
            {
                bLED = !bLED;
                digitalWrite(LED_BUILTIN, bLED);

                if (charArrayNonAnimation[i].width <= 50 || charArrayNonAnimation[i].height <= 50)
                {
                    u8g2.clearBuffer();
                    u8g2.home();
                    u8g2.setCursor(3, oled_LineH * 1 + 2);
                    u8g2.print(charArrayNonAnimation[i].name);
                    u8g2.sendBuffer();
                    smallFramechar(charGraphicsBuffer);
                }
                else
                {
                    display.drawBitmap(0, 0, charGraphicsBuffer, charArrayNonAnimation[i].width, charArrayNonAnimation[i].height, 1);
                    display.display();
                }
                effectime--;
                Serial.println(effectime);
            }
            delete[] charGraphicsBuffer;
        }
        delay(1000);
    }; // end charArray_NonAnim function (to display all the char graphics loaded)

}; // end class charArrayNonAnim

// charArrayNonAnim charNoAnimation; // need an object to call the function from charArrayAnim.h

// charArrayNonAnim::charArrayNonAnim(/* args */)
// {
// }

// charArrayNonAnim::~charArrayNonAnim()
// {
// }

// =========================================================
// END OF PROGRAM
// =========================================================
