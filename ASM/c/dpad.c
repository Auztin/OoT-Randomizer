#include "gfx.h"
#include "dpad.h"

extern uint8_t CFG_DISPLAY_DPAD;

//unknown 00 is a pointer to some vector transformation when the sound is tied to an actor. actor + 0x3E, when not tied to an actor (map), always 80104394
//unknown 01 is always 4 in my testing
//unknown 02 is a pointer to some kind of audio configuration Always 801043A0 in my testing
//unknown 03 is always a3 in my testing
//unknown 04 is always a3 + 0x08 in my testing (801043A8)
typedef void(*playsfx_t)(uint16_t sfx, z64_xyzf_t *unk_00_, int8_t unk_01_ , float *unk_02_, float *unk_03_, float *unk_04_);
typedef void(*usebutton_t)(z64_game_t *game, z64_link_t *link, uint8_t item, uint8_t button);

#define z64_playsfx   ((playsfx_t)      0x800C806C)
#define z64_usebutton ((usebutton_t)    0x8038C9A0)

uint8_t* c_swaps = (uint8_t*)&(z64_file.scene_flags[0x2A].unk_00_);

// uint8_t* adult_swap_c_items = ((uint8_t*)&(z64_file.scene_flags[0x2A].unk_00_))+1;
// uint8_t* child_swap_c_items = ((uint8_t*)&(z64_file.scene_flags[0x2B].unk_00_))+1;
// uint8_t was_child_trade_slot[3] = {0, 0, 0};

void handle_swap_c_items(uint8_t* swap) {
    if (swap[0] == 0 && swap[1] == 0 && swap[2] == 0) {
        swap[0] = 0xFF;
        swap[1] = 0xFF;
        swap[2] = 0xFF;
        swap[3] = 0xFF;
        swap[4] = 0xFF;
        swap[5] = 0xFF;
    }
    uint8_t buffer_buttons[3];
    uint8_t buffer_items[3];
    for (uint8_t i = 0; i < 3; i++) {
        buffer_buttons[i] = z64_file.c_button_slots[i];
        buffer_items[i] = z64_file.button_items[i+1];

        switch ((uint8_t)z64_file.items[swap[i]]) {
            case 0xFF:
            case 0x2C:
                swap[i] = 0xFF;
                swap[i+3] = 0xFF;
        }
        z64_file.c_button_slots[i] = swap[i];
        z64_file.button_items[i+1] = swap[i] == 0xFF ? 0xFF : z64_file.items[swap[i]];

        if (swap[i] != 0xFF) {
            for (uint8_t slot = 0; slot < 24; slot++) {
                uint8_t item = z64_file.items[slot];
                switch (item) {
                    case 0x04:
                        item = 0x38;
                        break;
                    case 0x0C:
                        item = 0x39;
                        break;
                    case 0x12:
                        item = 0x3A;
                        break;
                }
                if (item == swap[i+3]) {
                    z64_file.button_items[i+1] = item;
                    break;
                }
            }
        }
    }

    if (swap[0] != 0xFF) z64_UpdateItemButton(&z64_game, 1);
    if (swap[1] != 0xFF) z64_UpdateItemButton(&z64_game, 2);
    if (swap[2] != 0xFF) z64_UpdateItemButton(&z64_game, 3);

    swap[0] = buffer_buttons[0];
    swap[1] = buffer_buttons[1];
    swap[2] = buffer_buttons[2];
    swap[3] = buffer_items[0];
    swap[4] = buffer_items[1];
    swap[5] = buffer_items[2];

    z64_playsfx(0x835, (z64_xyzf_t*)0x80104394, 0x04, (float*)0x801043A0, (float*)0x801043A0, (float*)0x801043A8);
}

// void handle_swap_c_items(uint8_t* swap) {
//     uint8_t buffer[3];
//     uint8_t buffer_child_trade_slot[3];
//     uint8_t is_child_trade_slot;
//     for (uint8_t i = 0; i < 3; i++) {
//         is_child_trade_slot = 0;
//         switch ((uint8_t)z64_file.button_items[i+1]) {
//             case 0x38:
//             case 0x39:
//             case 0x3A:
//                 buffer[i] = z64_file.button_items[i+1];
//                 break;
//             case 0x23:
//             case 0x24:
//             case 0x25:
//             case 0x26:
//             case 0x27:
//             case 0x28:
//             case 0x29:
//             case 0x2A:
//             case 0x2B:
//             case 0x2C:
//                 is_child_trade_slot = 1;
//             default:
//                 if (is_child_trade_slot) buffer_child_trade_slot[i] = 1;
//                 else buffer_child_trade_slot[i] = 0;
//                 buffer[i] = z64_file.c_button_slots[i];
//         }

//         switch (swap[i]) {
//             case 0x38:
//             case 0x39:
//             case 0x3A:
//                 z64_file.c_button_slots[i] = 0x03;
//                 z64_file.button_items[i+1] = swap[i];
//                 break;
//             default:
//                 z64_file.c_button_slots[i] = swap[i];
//                 if (z64_file.link_age == 0 && was_child_trade_slot[i]) z64_file.button_items[i+1] = z64_file.items[Z64_SLOT_CHILD_TRADE];
//                 else z64_file.button_items[i+1] = swap[i] == 0xFF ? 0xFF : z64_file.items[swap[i]];
//         }
//     }

//     if (swap[0] != 0xFF) z64_UpdateItemButton(&z64_game, 1);
//     if (swap[1] != 0xFF) z64_UpdateItemButton(&z64_game, 2);
//     if (swap[2] != 0xFF) z64_UpdateItemButton(&z64_game, 3);

//     swap[0] = buffer[0];
//     swap[1] = buffer[1];
//     swap[2] = buffer[2];
//     if (z64_file.link_age == 0) {
//         was_child_trade_slot[0] = buffer_child_trade_slot[0];
//         was_child_trade_slot[1] = buffer_child_trade_slot[1];
//         was_child_trade_slot[2] = buffer_child_trade_slot[2];
//     }

//     z64_playsfx(0x835, (z64_xyzf_t*)0x80104394, 0x04, (float*)0x801043A0, (float*)0x801043A0, (float*)0x801043A8);
// }

void handle_dpad() {

    pad_t pad_pressed = z64_game.common.input[0].pad_pressed;
    pad_t pad_held = z64_ctxt.input[0].raw.pad;

    if (CAN_USE_DPAD && DISPLAY_DPAD && (!pad_held.a || !CAN_DRAW_DUNGEON_INFO)) {
        if (z64_file.link_age == 0) {
            if (pad_pressed.dl && z64_file.iron_boots) {
                if (z64_file.equip_boots == 2) z64_file.equip_boots = 1;
                else z64_file.equip_boots = 2;
                z64_UpdateEquipment(&z64_game, &z64_link);
                z64_playsfx(0x835, (z64_xyzf_t*)0x80104394, 0x04, (float*)0x801043A0, (float*)0x801043A0, (float*)0x801043A8);
            }

            if (pad_pressed.dr && z64_file.hover_boots) {
                if (z64_file.equip_boots == 3) z64_file.equip_boots = 1;
                else z64_file.equip_boots = 3;
                z64_UpdateEquipment(&z64_game, &z64_link);
                z64_playsfx(0x835, (z64_xyzf_t*)0x80104394, 0x04, (float*)0x801043A0, (float*)0x801043A0, (float*)0x801043A8);
            }

            if (pad_pressed.du) {
                handle_swap_c_items(c_swaps);
            }
        }

        if (z64_file.link_age == 1) {
            if (pad_pressed.dr && CAN_USE_CHILD_TRADE) {
                z64_usebutton(&z64_game,&z64_link,z64_file.items[Z64_SLOT_CHILD_TRADE], 2);
            }

            if (pad_pressed.du) {
                handle_swap_c_items(c_swaps+6);
            }
        }

        if (pad_pressed.dd && CAN_USE_OCARINA) {
            z64_usebutton(&z64_game,&z64_link,z64_file.items[Z64_SLOT_OCARINA], 2);
        }
    }
}

void draw_dpad() {
    z64_disp_buf_t *db = &(z64_ctxt.gfx->overlay);
    if (CAN_DRAW_DUNGEON_INFO || (DISPLAY_DPAD && CFG_DISPLAY_DPAD)) {
        gSPDisplayList(db->p++, &setup_db);
        gDPPipeSync(db->p++);
        gDPSetCombineMode(db->p++, G_CC_MODULATEIA_PRIM, G_CC_MODULATEIA_PRIM);
        uint16_t alpha = z64_game.hud_alpha_channels.rupees_keys_magic;

        gDPSetPrimColor(db->p++, 0, 0, 0xFF, 0xFF, 0xFF, alpha);
        sprite_load(db, &dpad_sprite, 0, 1);
        sprite_draw(db, &dpad_sprite, 0, 271, 64, 16, 16);

        if (CAN_DRAW_DUNGEON_INFO && CFG_DPAD_DUNGEON_INFO_ENABLE) {
            // Zora sapphire on D-down
            sprite_load(db, &stones_sprite, 2, 1);
            sprite_draw(db, &stones_sprite, 0, 273, 77, 12, 12);

            // small key on D-right
            sprite_load(db, &quest_items_sprite, 17, 1);
            sprite_draw(db, &quest_items_sprite, 0, 285, 66, 12, 12);

            // map on D-left
            sprite_load(db, &quest_items_sprite, 16, 1);
            sprite_draw(db, &quest_items_sprite, 0, 260, 66, 12, 12);
        } else {
            if (!CAN_USE_DPAD)
                gDPSetPrimColor(db->p++, 0, 0, 0xFF, 0xFF, 0xFF, alpha * 0x46 / 0xFF);

            if (z64_file.iron_boots && z64_file.link_age==0) {
                sprite_load(db, &items_sprite, 69, 1);
                if (z64_file.equip_boots == 2) {
                    sprite_draw(db, &items_sprite, 0, 258, 64, 16, 16);
                }
                else {
                    sprite_draw(db, &items_sprite, 0, 260, 66, 12, 12);
                }
            }

            if (z64_file.hover_boots && z64_file.link_age == 0) {
                sprite_load(db, &items_sprite, 70, 1);
                if (z64_file.equip_boots == 3) {
                    sprite_draw(db, &items_sprite, 0, 283, 64, 16, 16);
                }
                else {
                    sprite_draw(db, &items_sprite, 0, 285, 66, 12, 12);
                }
            }

            if (z64_file.items[Z64_SLOT_CHILD_TRADE] >= Z64_ITEM_WEIRD_EGG && z64_file.items[Z64_SLOT_CHILD_TRADE] <= Z64_ITEM_MASK_OF_TRUTH && z64_file.link_age == 1) {
                if(!CAN_USE_DPAD || !CAN_USE_CHILD_TRADE) gDPSetPrimColor(db->p++, 0, 0, 0xFF, 0xFF, 0xFF, alpha * 0x46 / 0xFF);
                else gDPSetPrimColor(db->p++, 0, 0, 0xFF, 0xFF, 0xFF, alpha);
                sprite_load(db, &items_sprite, z64_file.items[Z64_SLOT_CHILD_TRADE], 1);
                sprite_draw(db, &items_sprite, 0, 285, 66, 12, 12);
            }

            if (z64_file.items[Z64_SLOT_OCARINA] == Z64_ITEM_FAIRY_OCARINA || z64_file.items[Z64_SLOT_OCARINA] == Z64_ITEM_OCARINA_OF_TIME) {
                if(!CAN_USE_DPAD || !CAN_USE_OCARINA) gDPSetPrimColor(db->p++, 0, 0, 0xFF, 0xFF, 0xFF, alpha * 0x46 / 0xFF);
                else gDPSetPrimColor(db->p++, 0, 0, 0xFF, 0xFF, 0xFF, alpha);
                sprite_load(db, &items_sprite, z64_file.items[Z64_SLOT_OCARINA], 1);
                sprite_draw(db, &items_sprite, 0, 273, 77, 12,12);
            }
        }

        gDPPipeSync(db->p++);
    }
}
