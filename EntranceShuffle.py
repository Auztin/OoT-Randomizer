import random
import logging
from Fill import ShuffleError
from collections import OrderedDict
from Search import Search
from Region import TimeOfDay
from Rules import set_entrances_based_rules
from Entrance import Entrance
from State import State
from Item import ItemFactory


def get_entrance_pool(type):
    return [entrance_data for entrance_data in entrance_shuffle_table if entrance_data[0] == type]


def entrance_instances(world, entrance_pool):
    entrance_instances = []
    for type, forward_entrance, *return_entrance in entrance_pool:
        forward_entrance = set_shuffled_entrance(world, forward_entrance[0], forward_entrance[1], type)
        forward_entrance.primary = True
        if return_entrance:
            return_entrance = return_entrance[0]
            return_entrance = set_shuffled_entrance(world, return_entrance[0], return_entrance[1], type)
            forward_entrance.bind_two_way(return_entrance)
        entrance_instances.append(forward_entrance)
    return entrance_instances


def set_shuffled_entrance(world, name, data, type):
    entrance = world.get_entrance(name)
    entrance.type = type
    entrance.data = data
    entrance.shuffled = True
    return entrance


def assume_pool_reachable(world, entrance_pool):
    assumed_pool = []
    for entrance in entrance_pool:
        assumed_forward = entrance.assume_reachable()
        if entrance.reverse != None:
            assumed_return = entrance.reverse.assume_reachable()
            if entrance.type in ('Dungeon', 'Interior', 'Grotto', 'Grave', 'SpecialGrave'):
                # Dungeon, Grotto/Grave and Simple Interior exits shouldn't be assumed to be able to give access to their parent region
                assumed_return.set_rule(lambda state, **kwargs: False)
            assumed_forward.bind_two_way(assumed_return)
        assumed_pool.append(assumed_forward)
    return assumed_pool

#   Abbreviations
#       DMC     Death Mountain Crater
#       DMT     Death Mountain Trail
#       GC      Goron City
#       GF      Gerudo Fortress
#       GS      Gold Skulltula
#       GV      Gerudo Valley
#       HC      Hyrule Castle
#       HF      Hyrule Field
#       KF      Kokiri Forest
#       LH      Lake Hylia
#       LLR     Lon Lon Ranch
#       LW      Lost Woods
#       OGC     Outside Ganon's Castle
#       SFM     Sacred Forest Meadow
#       ToT     Temple of Time
#       ZD      Zora's Domain
#       ZF      Zora's Fountain
#       ZR      Zora's River

entrance_shuffle_table = [
    ('Dungeon',         ('KF Outside Deku Tree -> Deku Tree Lobby',                         { 'index': 0x0000 }),
                        ('Deku Tree Lobby -> KF Outside Deku Tree',                         { 'index': 0x0209, 'blue_warp': 0x0457 })),
    ('Dungeon',         ('Dodongos Cavern Entryway -> Dodongos Cavern Beginning',           { 'index': 0x0004 }),
                        ('Dodongos Cavern Beginning -> Dodongos Cavern Entryway',           { 'index': 0x0242, 'blue_warp': 0x047A })),
    ('Dungeon',         ('Zoras Fountain -> Jabu Jabus Belly Beginning',                    { 'index': 0x0028 }),
                        ('Jabu Jabus Belly Beginning -> Zoras Fountain',                    { 'index': 0x0221, 'blue_warp': 0x010E })),
    ('Dungeon',         ('Sacred Forest Meadow -> Forest Temple Lobby',                     { 'index': 0x0169 }),
                        ('Forest Temple Lobby -> Sacred Forest Meadow',                     { 'index': 0x0215, 'blue_warp': 0x0608 })),
    ('Dungeon',         ('Fire Temple Entrance -> Fire Temple Lower',                       { 'index': 0x0165 }),
                        ('Fire Temple Lower -> Fire Temple Entrance',                       { 'index': 0x024A, 'blue_warp': 0x0564 })),
    ('Dungeon',         ('Lake Hylia -> Water Temple Lobby',                                { 'index': 0x0010 }),
                        ('Water Temple Lobby -> Lake Hylia',                                { 'index': 0x021D, 'blue_warp': 0x060C })),
    ('Dungeon',         ('Desert Colossus -> Spirit Temple Lobby',                          { 'index': 0x0082 }),
                        ('Spirit Temple Lobby -> Desert Colossus',                          { 'index': 0x01E1, 'blue_warp': 0x0610 })),
    ('Dungeon',         ('Graveyard Warp Pad Region -> Shadow Temple Entryway',             { 'index': 0x0037 }),
                        ('Shadow Temple Entryway -> Graveyard Warp Pad Region',             { 'index': 0x0205, 'blue_warp': 0x0580 })),
    ('Dungeon',         ('Kakariko Village -> Bottom of the Well',                          { 'index': 0x0098 }),
                        ('Bottom of the Well -> Kakariko Village',                          { 'index': 0x02A6 })),
    ('Dungeon',         ('Zoras Fountain -> Ice Cavern Beginning',                          { 'index': 0x0088 }),
                        ('Ice Cavern Beginning -> Zoras Fountain',                          { 'index': 0x03D4 })),
    ('Dungeon',         ('Gerudo Fortress -> Gerudo Training Grounds Lobby',                { 'index': 0x0008 }),
                        ('Gerudo Training Grounds Lobby -> Gerudo Fortress',                { 'index': 0x03A8 })),

    ('Interior',        ('Kokiri Forest -> KF Midos House',                                 { 'index': 0x0433 }),
                        ('KF Midos House -> Kokiri Forest',                                 { 'index': 0x0443 })),
    ('Interior',        ('Kokiri Forest -> KF Sarias House',                                { 'index': 0x0437 }),
                        ('KF Sarias House -> Kokiri Forest',                                { 'index': 0x0447 })),
    ('Interior',        ('Kokiri Forest -> KF House of Twins',                              { 'index': 0x009C }),
                        ('KF House of Twins -> Kokiri Forest',                              { 'index': 0x033C })),
    ('Interior',        ('Kokiri Forest -> KF Know It All House',                           { 'index': 0x00C9 }),
                        ('KF Know It All House -> Kokiri Forest',                           { 'index': 0x026A })),
    ('Interior',        ('Kokiri Forest -> KF Kokiri Shop',                                 { 'index': 0x00C1 }),
                        ('KF Kokiri Shop -> Kokiri Forest',                                 { 'index': 0x0266 })),
    ('Interior',        ('Lake Hylia -> LH Lab',                                            { 'index': 0x0043 }),
                        ('LH Lab -> Lake Hylia',                                            { 'index': 0x03CC })),
    ('Interior',        ('Lake Hylia -> LH Fishing Hole',                                   { 'index': 0x045F }),
                        ('LH Fishing Hole -> Lake Hylia',                                   { 'index': 0x0309 })),
    ('Interior',        ('GV Fortress Side -> GV Carpenter Tent',                           { 'index': 0x03A0 }),
                        ('GV Carpenter Tent -> GV Fortress Side',                           { 'index': 0x03D0 })),
    ('Interior',        ('Market Entrance -> Market Guard House',                           { 'index': 0x007E }),
                        ('Market Guard House -> Market Entrance',                           { 'index': 0x026E })),
    ('Interior',        ('Market -> Market Mask Shop',                                      { 'index': 0x0530 }),
                        ('Market Mask Shop -> Market',                                      { 'index': 0x01D1 })),
    ('Interior',        ('Market -> Market Bombchu Bowling',                                { 'index': 0x0507 }),
                        ('Market Bombchu Bowling -> Market',                                { 'index': 0x03BC })),
    ('Interior',        ('Market -> Market Potion Shop',                                    { 'index': 0x0388 }),
                        ('Market Potion Shop -> Market',                                    { 'index': 0x02A2 })),
    ('Interior',        ('Market -> Market Treasure Chest Game',                            { 'index': 0x0063 }),
                        ('Market Treasure Chest Game -> Market',                            { 'index': 0x01D5 })),
    ('Interior',        ('Market -> Market Bombchu Shop',                                   { 'index': 0x0528 }),
                        ('Market Bombchu Shop -> Market',                                   { 'index': 0x03C0 })),
    ('Interior',        ('Market -> Market Man in Green House',                             { 'index': 0x043B }),
                        ('Market Man in Green House -> Market',                             { 'index': 0x0067 })),
    ('Interior',        ('Kakariko Village -> Kak Carpenter Boss House',                    { 'index': 0x02FD }),
                        ('Kak Carpenter Boss House -> Kakariko Village',                    { 'index': 0x0349 })),
    ('Interior',        ('Kakariko Village -> Kak House of Skulltula',                      { 'index': 0x0550 }),
                        ('Kak House of Skulltula -> Kakariko Village',                      { 'index': 0x04EE })),
    ('Interior',        ('Kakariko Village -> Kak Impas House',                             { 'index': 0x039C }),
                        ('Kak Impas House -> Kakariko Village',                             { 'index': 0x0345 })),
    ('Interior',        ('Kak Impas Ledge -> Kak Impas House Back',                         { 'index': 0x05C8 }),
                        ('Kak Impas House Back -> Kak Impas Ledge',                         { 'index': 0x05DC })),
    ('Interior',        ('Kak Backyard -> Kak Odd Medicine Building',                       { 'index': 0x0072 }),
                        ('Kak Odd Medicine Building -> Kak Backyard',                       { 'index': 0x034D })),
    ('Interior',        ('Graveyard -> Graveyard Dampes House',                             { 'index': 0x030D }),
                        ('Graveyard Dampes House -> Graveyard',                             { 'index': 0x0355 })),
    ('Interior',        ('Goron City -> GC Shop',                                           { 'index': 0x037C }),
                        ('GC Shop -> Goron City',                                           { 'index': 0x03FC })),
    ('Interior',        ('Zoras Domain -> ZD Shop',                                         { 'index': 0x0380 }),
                        ('ZD Shop -> Zoras Domain',                                         { 'index': 0x03C4 })),
    ('Interior',        ('Lon Lon Ranch -> LLR Talons House',                               { 'index': 0x004F }),
                        ('LLR Talons House -> Lon Lon Ranch',                               { 'index': 0x0378 })),
    ('Interior',        ('Lon Lon Ranch -> LLR Stables',                                    { 'index': 0x02F9 }),
                        ('LLR Stables -> Lon Lon Ranch',                                    { 'index': 0x042F })),
    ('Interior',        ('Lon Lon Ranch -> LLR Tower',                                      { 'index': 0x05D0 }),
                        ('LLR Tower -> Lon Lon Ranch',                                      { 'index': 0x05D4 })),
    ('Interior',        ('Market -> Market Bazaar',                                         { 'index': 0x052C }),
                        ('Market Bazaar -> Market',                                         { 'index': 0x03B8, 'dynamic_address': 0xBEFD74 })),
    ('Interior',        ('Market -> Market Shooting Gallery',                               { 'index': 0x016D }),
                        ('Market Shooting Gallery -> Market',                               { 'index': 0x01CD, 'dynamic_address': 0xBEFD7C })),
    ('Interior',        ('Kakariko Village -> Kak Bazaar',                                  { 'index': 0x00B7 }),
                        ('Kak Bazaar -> Kakariko Village',                                  { 'index': 0x0201, 'dynamic_address': 0xBEFD72 })),
    ('Interior',        ('Kakariko Village -> Kak Shooting Gallery',                        { 'index': 0x003B }),
                        ('Kak Shooting Gallery -> Kakariko Village',                        { 'index': 0x0463, 'dynamic_address': 0xBEFD7A })),
    ('Interior',        ('Desert Colossus -> Colossus Great Fairy Fountain',                { 'index': 0x0588 }),
                        ('Colossus Great Fairy Fountain -> Desert Colossus',                { 'index': 0x057C, 'dynamic_address': 0xBEFD82 })),
    ('Interior',        ('Hyrule Castle Grounds -> HC Great Fairy Fountain',                { 'index': 0x0578 }),
                        ('HC Great Fairy Fountain -> Castle Grounds',                       { 'index': 0x0340, 'dynamic_address': 0xBEFD80 })),
    ('Interior',        ('Ganons Castle Grounds -> OGC Great Fairy Fountain',               { 'index': 0x04C2 }),
                        ('OGC Great Fairy Fountain -> Castle Grounds',                      { 'index': 0x0340, 'dynamic_address': 0xBEFD6C })),
    ('Interior',        ('DMC Lower Nearby -> DMC Great Fairy Fountain',                    { 'index': 0x04BE }),
                        ('DMC Great Fairy Fountain -> DMC Lower Local',                     { 'index': 0x0482, 'dynamic_address': 0xBEFD6A })),
    ('Interior',        ('Death Mountain Summit -> DMT Great Fairy Fountain',               { 'index': 0x0315 }),
                        ('DMT Great Fairy Fountain -> Death Mountain Summit',               { 'index': 0x045B, 'dynamic_address': 0xBEFD68 })),
    ('Interior',        ('Zoras Fountain -> ZF Great Fairy Fountain',                       { 'index': 0x0371 }),
                        ('ZF Great Fairy Fountain -> Zoras Fountain',                       { 'index': 0x0394, 'dynamic_address': 0xBEFD7E })),

    ('SpecialInterior', ('Kokiri Forest -> KF Links House',                                 { 'index': 0x0272 }),
                        ('KF Links House -> Kokiri Forest',                                 { 'index': 0x0211 })),
    ('SpecialInterior', ('ToT Entrance -> Temple of Time',                                  { 'index': 0x0053 }),
                        ('Temple of Time -> ToT Entrance',                                  { 'index': 0x0472 })),
    ('SpecialInterior', ('Kakariko Village -> Kak Windmill',                                { 'index': 0x0453 }),
                        ('Kak Windmill -> Kakariko Village',                                { 'index': 0x0351 })),
    ('SpecialInterior', ('Kakariko Village -> Kak Potion Shop Front',                       { 'index': 0x0384 }),
                        ('Kak Potion Shop Front -> Kakariko Village',                       { 'index': 0x044B })),
    ('SpecialInterior', ('Kak Backyard -> Kak Potion Shop Back',                            { 'index': 0x03EC }),
                        ('Kak Potion Shop Back -> Kak Backyard',                            { 'index': 0x04FF })),

    ('Grotto',          ('Desert Colossus -> Colossus Grotto',                              { 'grotto_id': 0x00, 'entrance': 0x05BC, 'content': 0xFD, 'scene': 0x5C }),
                        ('Colossus Grotto -> Desert Colossus',                              { 'grotto_id': 0x00 })),
    ('Grotto',          ('Lake Hylia -> LH Grotto',                                         { 'grotto_id': 0x01, 'entrance': 0x05A4, 'content': 0xEF, 'scene': 0x57 }),
                        ('LH Grotto -> Lake Hylia',                                         { 'grotto_id': 0x01 })),
    ('Grotto',          ('Zora River -> ZR Storms Grotto',                                  { 'grotto_id': 0x02, 'entrance': 0x05BC, 'content': 0xEB, 'scene': 0x54 }),
                        ('ZR Storms Grotto -> Zora River',                                  { 'grotto_id': 0x02 })),
    ('Grotto',          ('Zora River -> ZR Fairy Grotto',                                   { 'grotto_id': 0x03, 'entrance': 0x036D, 'content': 0xE6, 'scene': 0x54 }),
                        ('ZR Fairy Grotto -> Zora River',                                   { 'grotto_id': 0x03 })),
    ('Grotto',          ('Zora River -> ZR Open Grotto',                                    { 'grotto_id': 0x04, 'entrance': 0x003F, 'content': 0x29, 'scene': 0x54 }),
                        ('ZR Open Grotto -> Zora River',                                    { 'grotto_id': 0x04 })),
    ('Grotto',          ('DMC Lower Nearby -> DMC Hammer Grotto',                           { 'grotto_id': 0x05, 'entrance': 0x05A4, 'content': 0xF9, 'scene': 0x61 }),
                        ('DMC Hammer Grotto -> DMC Lower Local',                            { 'grotto_id': 0x05 })),
    ('Grotto',          ('DMC Upper Nearby -> DMC Upper Grotto',                            { 'grotto_id': 0x06, 'entrance': 0x003F, 'content': 0x7A, 'scene': 0x61 }),
                        ('DMC Upper Grotto -> DMC Upper Local',                             { 'grotto_id': 0x06 })),
    ('Grotto',          ('Goron City -> GC Grotto',                                         { 'grotto_id': 0x07, 'entrance': 0x05A4, 'content': 0xFB, 'scene': 0x62 }),
                        ('GC Grotto -> Goron City',                                         { 'grotto_id': 0x07 })),
    ('Grotto',          ('Death Mountain -> DMT Storms Grotto',                             { 'grotto_id': 0x08, 'entrance': 0x003F, 'content': 0x57, 'scene': 0x60 }),
                        ('DMT Storms Grotto -> Death Mountain',                             { 'grotto_id': 0x08 })),
    ('Grotto',          ('Death Mountain Summit -> DMT Cow Grotto',                         { 'grotto_id': 0x09, 'entrance': 0x05FC, 'content': 0xF8, 'scene': 0x60 }),
                        ('DMT Cow Grotto -> Death Mountain Summit',                         { 'grotto_id': 0x09 })),
    ('Grotto',          ('Kak Backyard -> Kak Open Grotto',                                 { 'grotto_id': 0x0A, 'entrance': 0x003F, 'content': 0x28, 'scene': 0x52 }),
                        ('Kak Open Grotto -> Kak Backyard',                                 { 'grotto_id': 0x0A })),
    ('Grotto',          ('Kakariko Village -> Kak Redead Grotto',                           { 'grotto_id': 0x0B, 'entrance': 0x05A0, 'content': 0xE7, 'scene': 0x52 }),
                        ('Kak Redead Grotto -> Kakariko Village',                           { 'grotto_id': 0x0B })),
    ('Grotto',          ('Hyrule Castle Grounds -> HC Storms Grotto',                       { 'grotto_id': 0x0C, 'entrance': 0x05B8, 'content': 0xF6, 'scene': 0x5F }),
                        ('HC Storms Grotto -> Hyrule Castle Grounds',                       { 'grotto_id': 0x0C })),
    ('Grotto',          ('Hyrule Field -> HF Tektite Grotto',                               { 'grotto_id': 0x0D, 'entrance': 0x05C0, 'content': 0xE1, 'scene': 0x51 }),
                        ('HF Tektite Grotto -> Hyrule Field',                               { 'grotto_id': 0x0D })),
    ('Grotto',          ('Hyrule Field -> HF Near Kak Grotto',                              { 'grotto_id': 0x0E, 'entrance': 0x0598, 'content': 0xE5, 'scene': 0x51 }),
                        ('HF Near Kak Grotto -> Hyrule Field',                              { 'grotto_id': 0x0E })),
    ('Grotto',          ('Hyrule Field -> HF Fairy Grotto',                                 { 'grotto_id': 0x0F, 'entrance': 0x036D, 'content': 0xFF, 'scene': 0x51 }),
                        ('HF Fairy Grotto -> Hyrule Field',                                 { 'grotto_id': 0x0F })),
    ('Grotto',          ('Hyrule Field -> HF Near Market Grotto',                           { 'grotto_id': 0x10, 'entrance': 0x003F, 'content': 0x00, 'scene': 0x51 }),
                        ('HF Near Market Grotto -> Hyrule Field',                           { 'grotto_id': 0x10 })),
    ('Grotto',          ('Hyrule Field -> HF Cow Grotto',                                   { 'grotto_id': 0x11, 'entrance': 0x05A8, 'content': 0xE4, 'scene': 0x51 }),
                        ('HF Cow Grotto -> Hyrule Field',                                   { 'grotto_id': 0x11 })),
    ('Grotto',          ('Hyrule Field -> HF Inside Fence Grotto',                          { 'grotto_id': 0x12, 'entrance': 0x059C, 'content': 0xE6, 'scene': 0x51 }),
                        ('HF Inside Fence Grotto -> Hyrule Field',                          { 'grotto_id': 0x12 })),
    ('Grotto',          ('Hyrule Field -> HF Open Grotto',                                  { 'grotto_id': 0x13, 'entrance': 0x003F, 'content': 0x03, 'scene': 0x51 }),
                        ('HF Open Grotto -> Hyrule Field',                                  { 'grotto_id': 0x13 })),
    ('Grotto',          ('Hyrule Field -> HF Southeast Grotto',                             { 'grotto_id': 0x14, 'entrance': 0x003F, 'content': 0x22, 'scene': 0x51 }),
                        ('HF Southeast Grotto -> Hyrule Field',                             { 'grotto_id': 0x14 })),
    ('Grotto',          ('Lon Lon Ranch -> LLR Grotto',                                     { 'grotto_id': 0x15, 'entrance': 0x05A4, 'content': 0xFC, 'scene': 0x63 }),
                        ('LLR Grotto -> Lon Lon Ranch',                                     { 'grotto_id': 0x15 })),
    ('Grotto',          ('SFM Entryway -> SFM Wolfos Grotto',                               { 'grotto_id': 0x16, 'entrance': 0x05B4, 'content': 0xED, 'scene': 0x56 }),
                        ('SFM Wolfos Grotto -> SFM Entryway',                               { 'grotto_id': 0x16 })),
    ('Grotto',          ('Sacred Forest Meadow -> SFM Storms Grotto',                       { 'grotto_id': 0x17, 'entrance': 0x05BC, 'content': 0xEE, 'scene': 0x56 }),
                        ('SFM Storms Grotto -> Sacred Forest Meadow',                       { 'grotto_id': 0x17 })),
    ('Grotto',          ('Sacred Forest Meadow -> SFM Fairy Grotto',                        { 'grotto_id': 0x18, 'entrance': 0x036D, 'content': 0xFF, 'scene': 0x56 }),
                        ('SFM Fairy Grotto -> Sacred Forest Meadow',                        { 'grotto_id': 0x18 })),
    ('Grotto',          ('LW Beyond Mido -> LW Scrubs Grotto',                              { 'grotto_id': 0x19, 'entrance': 0x05B0, 'content': 0xF5, 'scene': 0x5B }),
                        ('LW Scrubs Grotto -> LW Beyond Mido',                              { 'grotto_id': 0x19 })),
    ('Grotto',          ('Lost Woods -> LW Near Shortcuts Grotto',                          { 'grotto_id': 0x1A, 'entrance': 0x003F, 'content': 0x14, 'scene': 0x5B }),
                        ('LW Near Shortcuts Grotto -> Lost Woods',                          { 'grotto_id': 0x1A })),
    ('Grotto',          ('Kokiri Forest -> KF Storms Grotto',                               { 'grotto_id': 0x1B, 'entrance': 0x003F, 'content': 0x2C, 'scene': 0x55 }),
                        ('KF Storms Grotto -> Kokiri Forest',                               { 'grotto_id': 0x1B })),
    ('Grotto',          ('Zoras Domain -> ZD Storms Grotto',                                { 'grotto_id': 0x1C, 'entrance': 0x036D, 'content': 0xFF, 'scene': 0x58 }),
                        ('ZD Storms Grotto -> Zoras Domain',                                { 'grotto_id': 0x1C })),
    ('Grotto',          ('Gerudo Fortress -> GF Storms Grotto',                             { 'grotto_id': 0x1D, 'entrance': 0x036D, 'content': 0xFF, 'scene': 0x5D }),
                        ('GF Storms Grotto -> Gerudo Fortress',                             { 'grotto_id': 0x1D })),
    ('Grotto',          ('GV Fortress Side -> GV Storms Grotto',                            { 'grotto_id': 0x1E, 'entrance': 0x05BC, 'content': 0xF0, 'scene': 0x5A }),
                        ('GV Storms Grotto -> GV Fortress Side',                            { 'grotto_id': 0x1E })),
    ('Grotto',          ('Gerudo Valley -> GV Octorok Grotto',                              { 'grotto_id': 0x1F, 'entrance': 0x05AC, 'content': 0xF2, 'scene': 0x5A }),
                        ('GV Octorok Grotto -> Gerudo Valley',                              { 'grotto_id': 0x1F })),
    ('Grotto',          ('LW Beyond Mido -> Deku Theater',                                  { 'grotto_id': 0x20, 'entrance': 0x05C4, 'content': 0xF3, 'scene': 0x5B }),
                        ('Deku Theater -> LW Beyond Mido',                                  { 'grotto_id': 0x20 })),

    ('Grave',           ('Graveyard -> Graveyard Shield Grave',                             { 'index': 0x004B }),
                        ('Graveyard Shield Grave -> Graveyard',                             { 'index': 0x035D })),
    ('Grave',           ('Graveyard -> Graveyard Heart Piece Grave',                        { 'index': 0x031C }),
                        ('Graveyard Heart Piece Grave -> Graveyard',                        { 'index': 0x0361 })),
    ('Grave',           ('Graveyard -> Graveyard Composers Grave',                          { 'index': 0x002D }),
                        ('Graveyard Composers Grave -> Graveyard',                          { 'index': 0x050B })),

    ('SpecialGrave',    ('Graveyard -> Graveyard Dampes Grave',                             { 'index': 0x044F }),
                        ('Graveyard Dampes Grave -> Graveyard',                             { 'index': 0x0359 })),

    ('Overworld',       ('Kokiri Forest -> LW Bridge From Forest',                          { 'index': 0x05E0 }),
                        ('LW Bridge -> Kokiri Forest',                                      { 'index': 0x020D })),
    ('Overworld',       ('Kokiri Forest -> Lost Woods',                                     { 'index': 0x011E }),
                        ('LW Forest Exit -> Kokiri Forest',                                 { 'index': 0x0286 })),
    ('Overworld',       ('Lost Woods -> GC Woods Warp',                                     { 'index': 0x04E2 }),
                        ('GC Woods Warp -> Lost Woods',                                     { 'index': 0x04D6 })),
    ('Overworld',       ('Lost Woods -> Zora River',                                        { 'index': 0x01DD }),
                        ('Zora River -> Lost Woods',                                        { 'index': 0x04DA })),
    ('Overworld',       ('LW Beyond Mido -> SFM Entryway',                                  { 'index': 0x00FC }),
                        ('SFM Entryway -> LW Beyond Mido',                                  { 'index': 0x01A9 })),
    ('Overworld',       ('LW Bridge -> Hyrule Field',                                       { 'index': 0x0185 }),
                        ('Hyrule Field -> LW Bridge',                                       { 'index': 0x04DE })),
    ('Overworld',       ('Hyrule Field -> Lake Hylia',                                      { 'index': 0x0102 }),
                        ('Lake Hylia -> Hyrule Field',                                      { 'index': 0x0189 })),
    ('Overworld',       ('Hyrule Field -> Gerudo Valley',                                   { 'index': 0x0117 }),
                        ('Gerudo Valley -> Hyrule Field',                                   { 'index': 0x018D })),
    ('Overworld',       ('Hyrule Field -> Market Entrance',                                 { 'index': 0x0276 }),
                        ('Market Entrance -> Hyrule Field',                                 { 'index': 0x01FD })),
    ('Overworld',       ('Hyrule Field -> Kakariko Village',                                { 'index': 0x00DB }),
                        ('Kakariko Village -> Hyrule Field',                                { 'index': 0x017D })),
    ('Overworld',       ('Hyrule Field -> ZR Front',                                        { 'index': 0x00EA }),
                        ('ZR Front -> Hyrule Field',                                        { 'index': 0x0181 })),
    ('Overworld',       ('Hyrule Field -> Lon Lon Ranch',                                   { 'index': 0x0157 }),
                        ('Lon Lon Ranch -> Hyrule Field',                                   { 'index': 0x01F9 })),
    ('Overworld',       ('Lake Hylia -> Zoras Domain',                                      { 'index': 0x0328 }),
                        ('Zoras Domain -> Lake Hylia',                                      { 'index': 0x0560 })),
    ('Overworld',       ('GV Fortress Side -> Gerudo Fortress',                             { 'index': 0x0129 }),
                        ('Gerudo Fortress -> GV Fortress Side',                             { 'index': 0x022D })),
    ('Overworld',       ('GF Outside Gate -> Wasteland Near Fortress',                      { 'index': 0x0130 }),
                        ('Wasteland Near Fortress -> GF Outside Gate',                      { 'index': 0x03AC })),
    ('Overworld',       ('Wasteland Near Colossus -> Desert Colossus',                      { 'index': 0x0123 }),
                        ('Desert Colossus -> Wasteland Near Colossus',                      { 'index': 0x0365 })),
    ('Overworld',       ('Market Entrance -> Market',                                       { 'index': 0x00B1 }),
                        ('Market -> Market Entrance',                                       { 'index': 0x0033 })),
    ('Overworld',       ('Market -> Castle Grounds',                                        { 'index': 0x0138 }),
                        ('Castle Grounds -> Market',                                        { 'index': 0x025A })),
    ('Overworld',       ('Market -> ToT Entrance',                                          { 'index': 0x0171 }),
                        ('ToT Entrance -> Market',                                          { 'index': 0x025E })),
    ('Overworld',       ('Kakariko Village -> Graveyard',                                   { 'index': 0x00E4 }),
                        ('Graveyard -> Kakariko Village',                                   { 'index': 0x0195 })),
    ('Overworld',       ('Kak Behind Gate -> Death Mountain',                               { 'index': 0x013D }),
                        ('Death Mountain -> Kak Behind Gate',                               { 'index': 0x0191 })),
    ('Overworld',       ('Death Mountain -> Goron City',                                    { 'index': 0x014D }),
                        ('Goron City -> Death Mountain',                                    { 'index': 0x01B9 })),
    ('Overworld',       ('GC Darunias Chamber -> DMC Lower Local',                          { 'index': 0x0246 }),
                        ('DMC Lower Nearby -> GC Darunias Chamber',                         { 'index': 0x01C1 })),
    ('Overworld',       ('Death Mountain Summit -> DMC Upper Local',                        { 'index': 0x0147 }),
                        ('DMC Upper Nearby -> Death Mountain Summit',                       { 'index': 0x01BD })),
    ('Overworld',       ('ZR Behind Waterfall -> Zoras Domain',                             { 'index': 0x0108 }),
                        ('Zoras Domain -> ZR Behind Waterfall',                             { 'index': 0x019D })),
    ('Overworld',       ('ZD Behind King Zora -> Zoras Fountain',                           { 'index': 0x0225 }),
                        ('Zoras Fountain -> ZD Behind King Zora',                           { 'index': 0x01A1 })),

    ('OwlDrop',         ('LH Owl Flight -> Hyrule Field',                                   { 'index': 0x027E, 'code_address': 0xAC9F26 })),
    ('OwlDrop',         ('DMT Owl Flight -> Kak Impas Ledge',                               { 'index': 0x0554, 'code_address': 0xAC9EF2 })),
]


class EntranceShuffleError(ShuffleError):
    pass


# Set entrances of all worlds, first initializing them to their default regions, then potentially shuffling part of them
def set_entrances(worlds):
    for world in worlds:
        world.initialize_entrances()

    if worlds[0].entrance_shuffle != 'off':
        shuffle_random_entrances(worlds)

    set_entrances_based_rules(worlds)


# Shuffles entrances that need to be shuffled in all worlds
def shuffle_random_entrances(worlds):

    # Store all locations reachable before shuffling to differentiate which locations were already unreachable from those we made unreachable
    complete_itempool = [item for world in worlds for item in world.get_itempool_with_dungeon_items()]
    max_search = Search.max_explore([world.state for world in worlds], complete_itempool)

    non_drop_locations = [location for world in worlds for location in world.get_locations() if location.type not in ('Drop', 'Event')]
    max_search.visit_locations(non_drop_locations)
    locations_to_ensure_reachable = list(filter(max_search.visited, non_drop_locations))

    # Shuffle all entrances within their own worlds
    for world in worlds:

        # Determine entrance pools based on settings, to be shuffled in the order we set them by
        entrance_pools = OrderedDict()

        if worlds[0].shuffle_special_indoor_entrances:
            entrance_pools['SpecialInterior'] = entrance_instances(world, get_entrance_pool('SpecialInterior'))

        if worlds[0].shuffle_overworld_entrances:
            entrance_pools['Overworld'] = entrance_instances(world, get_entrance_pool('Overworld'))
            # Overworld entrances should be shuffled from both directions, unlike other types of entrances
            for entrance in entrance_pools['Overworld'].copy():
                entrance.reverse.primary = True
                entrance_pools['Overworld'].append(entrance.reverse)
            entrance_pools['OwlDrop'] = entrance_instances(world, get_entrance_pool('OwlDrop'))

        if worlds[0].shuffle_dungeon_entrances:
            entrance_pools['Dungeon'] = entrance_instances(world, get_entrance_pool('Dungeon'))
            # The fill algorithm will already make sure gohma is reachable, however it can end up putting
            # a forest escape via the hands of spirit on Deku leading to Deku on spirit in logic. This is
            # not really a closed forest anymore, so specifically remove Deku Tree from closed forest.
            if worlds[0].open_forest == 'closed':
                entrance_pools['Dungeon'].remove(world.get_entrance('KF Outside Deku Tree -> Deku Tree Lobby'))
                world.get_entrance('KF Outside Deku Tree -> Deku Tree Lobby').shuffled = False
                world.get_entrance('Deku Tree Lobby -> KF Outside Deku Tree').shuffled = False

        if worlds[0].shuffle_interior_entrances:
            entrance_pools['Interior'] = entrance_instances(world, get_entrance_pool('Interior')) + entrance_pools.get('SpecialInterior', [])

        if worlds[0].shuffle_grotto_entrances:
            entrance_pools['GrottoGrave'] = entrance_instances(world, get_entrance_pool('Grotto') + get_entrance_pool('Grave'))
            if worlds[0].shuffle_special_indoor_entrances:
                entrance_pools['GrottoGrave'] += entrance_instances(world, get_entrance_pool('SpecialGrave'))

        # Set the assumption that all entrances are reachable
        target_entrance_pools = {}
        for pool_type, entrance_pool in entrance_pools.items():
            target_entrance_pools[pool_type] = assume_pool_reachable(world, entrance_pool)

        # Special interiors need to be handled specifically by placing them in reverse and among all interiors, including normal ones
        if 'SpecialInterior' in entrance_pools:
            entrance_pools['SpecialInterior'] = [entrance.reverse for entrance in entrance_pools['SpecialInterior']]
            target_entrance_pools['SpecialInterior'] = [entrance.reverse for entrance in target_entrance_pools['Interior']]

        # Owl Drops are extra entrances that will be connected to an owl drop or will be a duplicate entrance to an overworld entrance
        # We don't assume they are reachable until placing them because we don't want the placement algorithm to expect all overworld regions to be reachable
        if 'OwlDrop' in entrance_pools:
            duplicate_overworld_targets = [target.copy(target.parent_region) for target in target_entrance_pools['Overworld']]
            for target in duplicate_overworld_targets:
                target.connect(world.get_region(target.connected_region))
                target.parent_region.exits.append(target)
            target_entrance_pools['OwlDrop'] += duplicate_overworld_targets
            for target in target_entrance_pools['OwlDrop']:
                target.set_rule(lambda state, **kwargs: False)

        # Set entrances defined in the distribution
        world.distribution.set_shuffled_entrances(worlds, entrance_pools, target_entrance_pools, locations_to_ensure_reachable, complete_itempool)

        # Shuffle all entrances among the pools to shuffle
        for pool_type, entrance_pool in entrance_pools.items():
            if pool_type == 'SpecialInterior':
                # When placing special interiors, we pre place ToT and Links House first, making sure the assumed access rules are always valid
                temple_of_time_exit = world.get_entrance('Temple of Time -> ToT Entrance')
                links_house_exit = world.get_entrance('KF Links House -> Kokiri Forest')
                for target in target_entrance_pools[pool_type]:
                    target.set_rule(lambda state, age=None, **kwargs: temple_of_time_exit.connected_region == None or (links_house_exit.connected_region == None and age == 'child'))
                shuffle_entrance_pool(worlds, [temple_of_time_exit], target_entrance_pools[pool_type], locations_to_ensure_reachable)
                shuffle_entrance_pool(worlds, [links_house_exit], target_entrance_pools[pool_type], locations_to_ensure_reachable)

            shuffle_entrance_pool(worlds, entrance_pool, target_entrance_pools[pool_type], locations_to_ensure_reachable)

            if pool_type == 'OwlDrop':
                # Delete all unused owl drop targets after placing the entrances, since the unused targets won't ever be replaced
                for target in target_entrance_pools[pool_type]:
                    delete_target_entrance(target)

    # Multiple checks after shuffling entrances to make sure everything went fine
    max_search = Search.max_explore([world.state for world in worlds], complete_itempool)

    # Check that all shuffled entrances are properly connected to a region
    for world in worlds:
        for entrance in world.get_shuffled_entrances():
            if entrance.connected_region == None:
                logging.getLogger('').error('%s was shuffled but still isn\'t connected to any region [World %d]', entrance, world.id)

    # Check for game beatability in all worlds
    if not max_search.can_beat_game(False):
        raise EntranceShuffleError('Cannot beat game!')

    # Validate the worlds one last time to ensure all special conditions are still valid
    try:
        validate_worlds(worlds, None, locations_to_ensure_reachable, complete_itempool)
    except EntranceShuffleError as error:
        raise EntranceShuffleError('Worlds are not valid after shuffling entrances, Reason: %s' % error)


# Shuffle all entrances within a provided pool
def shuffle_entrance_pool(worlds, entrance_pool, target_entrances, locations_to_ensure_reachable, retry_count=20):

    # Split entrances between those that have requirements (restrictive) and those that do not (soft). These are primarily age or time of day requirements.
    restrictive_entrances, soft_entrances = split_entrances_by_requirements(worlds, entrance_pool, target_entrances)

    while retry_count:
        retry_count -= 1
        rollbacks = []

        try:
            # Shuffle restrictive entrances first while more regions are available in order to heavily reduce the chances of the placement failing.
            shuffle_entrances(worlds, restrictive_entrances, target_entrances, rollbacks, locations_to_ensure_reachable)

            # Shuffle the rest of the entrances, we don't have to check for beatability or reachability of locations when placing those
            shuffle_entrances(worlds, soft_entrances, target_entrances, rollbacks)

            # Fully validate the resulting worlds to ensure everything is still fine after shuffling this pool
            complete_itempool = [item for world in worlds for item in world.get_itempool_with_dungeon_items()]
            validate_worlds(worlds, None, locations_to_ensure_reachable, complete_itempool)

            # If all entrances could be connected without issues, log connections and continue
            for entrance, target in rollbacks:
                confirm_replacement(entrance, target)
            return

        except EntranceShuffleError as error:
            for entrance, target in rollbacks:
                restore_connections(entrance, target)
            logging.getLogger('').info('Failed to place all entrances in a pool for world %d. Will retry %d more times', entrance_pool[0].world.id, retry_count)
            logging.getLogger('').info('\t%s' % error)

    raise EntranceShuffleError('Entrance placement attempt count exceeded for world %d' % entrance_pool[0].world.id)


# Split entrances based on their requirements to figure out how each entrance should be handled when shuffling them
def split_entrances_by_requirements(worlds, entrances_to_split, assumed_entrances):

    # First, disconnect all root assumed entrances and save which regions they were originally connected to, so we can reconnect them later
    original_connected_regions = {}
    entrances_to_disconnect = set(assumed_entrances).union(entrance.reverse for entrance in assumed_entrances if entrance.reverse)
    for entrance in entrances_to_disconnect:
        if entrance.connected_region:
            original_connected_regions[entrance] = entrance.disconnect()

    # Generate the states with all assumed entrances disconnected
    # This ensures no assumed entrances corresponding to those we are shuffling are required in order for an entrance to be reachable as some age/tod
    complete_itempool = [item for world in worlds for item in world.get_itempool_with_dungeon_items()]
    max_search = Search.max_explore([world.state for world in worlds], complete_itempool)

    restrictive_entrances = []
    soft_entrances = []

    for entrance in entrances_to_split:
        # Here, we find entrances that may be unreachable under certain conditions
        if not max_search.spot_access(entrance, age='both', tod=TimeOfDay.ALL):
            restrictive_entrances.append(entrance)
            continue
        # If an entrance is reachable as both ages and all times of day with all the other entrances disconnected,
        # then it can always be made accessible in all situations by the Fill algorithm, no matter which combination of entrances we end up with.
        # Thus, those entrances aren't bound to any specific requirements and are very versatile during placement.
        soft_entrances.append(entrance)

    # Reconnect all disconnected entrances afterwards
    for entrance in entrances_to_disconnect:
        if entrance in original_connected_regions:
            entrance.connect(original_connected_regions[entrance])

    return restrictive_entrances, soft_entrances


# Shuffle entrances by placing them instead of entrances in the provided target entrances list
# While shuffling entrances, the algorithm will ensure worlds are still valid based on multiple criterias
def shuffle_entrances(worlds, entrances, target_entrances, rollbacks, locations_to_ensure_reachable=[]):

    # Retrieve all items in the itempool, all worlds included
    complete_itempool = [item for world in worlds for item in world.get_itempool_with_dungeon_items()]

    random.shuffle(entrances)

    # Place all entrances in the pool, validating worlds during every placement
    for entrance in entrances:
        if entrance.connected_region != None:
            continue
        random.shuffle(target_entrances)

        for target in target_entrances:
            if target.connected_region == None:
                continue

            # An entrance shouldn't be connected to its own scene, so we fail in that situation
            if entrance.parent_region.scene and entrance.parent_region.scene == target.connected_region.scene:
                logging.getLogger('').debug('Failed to connect %s To %s (Reason: Self scene connections are forbidden) [World %d]',
                                            entrance, target.connected_region, entrance.world.id)
                continue

            change_connections(entrance, target)

            try:
                validate_worlds(worlds, entrance, locations_to_ensure_reachable, complete_itempool)
                rollbacks.append((entrance, target))
                break
            except EntranceShuffleError as error:
                # If the entrance can't be placed there, log a debug message and change the connections back to what they were previously
                logging.getLogger('').debug('Failed to connect %s To %s (Reason: %s) [World %d]',
                                            entrance, entrance.connected_region, error, entrance.world.id)
                restore_connections(entrance, target)

        if entrance.connected_region == None:
            raise EntranceShuffleError('No more valid entrances to replace with %s in world %d' % (entrance, entrance.world.id))


# Validate the provided worlds' structures, raising an error if it's not valid based on our criterias
def validate_worlds(worlds, entrance_placed, locations_to_ensure_reachable, itempool):

    max_search = None

    if locations_to_ensure_reachable:
        max_search = Search.max_explore([world.state for world in worlds], itempool)
        # If ALR is enabled, ensure all locations we want to keep reachable are indeed still reachable 
        # Otherwise, just continue if the game is still beatable
        if not (worlds[0].check_beatable_only and max_search.can_beat_game(False)):
            max_search.visit_locations(locations_to_ensure_reachable)
            for location in locations_to_ensure_reachable:
                if not max_search.visited(location):
                    raise EntranceShuffleError('%s is unreachable' % location.name)

    if (entrance_placed == None and worlds[0].shuffle_special_indoor_entrances) or \
       (entrance_placed != None and entrance_placed.type in ['SpecialInterior', 'Overworld']):
        if max_search == None:
            max_search = Search.max_explore([world.state for world in worlds], itempool)

        for world in worlds:
            # Links House entrance should be reachable as child at some point in the seed
            links_house_entrance = get_entrance_replacing(world.get_region('KF Links House'), 'Kokiri Forest -> KF Links House')
            if not max_search.spot_access(links_house_entrance, age='child'):
                raise EntranceShuffleError('Links House Entrance is never reachable as child')

            # Temple of Time entrance should be reachable as both ages at some point in the seed
            temple_of_time_entrance = get_entrance_replacing(world.get_region('Temple of Time'), 'ToT Entrance -> Temple of Time')
            if not max_search.spot_access(temple_of_time_entrance, age='both'):
                raise EntranceShuffleError('Temple of Time Entrance is never reachable as both ages')

            # Windmill door entrance should be reachable as both ages at some point in the seed
            windmill_door_entrance = get_entrance_replacing(world.get_region('Kak Windmill'), 'Kakariko Village -> Kak Windmill')
            if not max_search.spot_access(windmill_door_entrance, age='both'):
                raise EntranceShuffleError('Windmill Door Entrance is never reachable as both ages')

            # Potion Shop front door should be reachable as both ages at some point in the seed
            potion_front_entrance = get_entrance_replacing(world.get_region('Kak Potion Shop Front'), 'Kakariko Village -> Kak Potion Shop Front')
            if not max_search.spot_access(potion_front_entrance, age='both'):
                raise EntranceShuffleError('Adult Potion Front Entrance is never reachable as both ages')

            # Potion Shop back door should be reachable as adult at some point in the seed
            potion_back_entrance = get_entrance_replacing(world.get_region('Kak Potion Shop Back'), 'Kak Backyard -> Kak Potion Shop Back')
            if not max_search.spot_access(potion_back_entrance, age='adult'):
                raise EntranceShuffleError('Adult Potion Back Entrance is never reachable as Adult')

            check_same_hint_region(potion_front_entrance, potion_back_entrance)

        # At least one valid starting region with all basic refills should be reachable without using any items at the beginning of the seed
        # Note this creates an empty State rather than reuse world.state (which already has starting items).
        no_items_search = Search([State(world) for world in worlds])

        valid_starting_regions = ['Kokiri Forest', 'Kakariko Village']
        for world in worlds:
            if not any(region for region in valid_starting_regions if no_items_search.can_reach(world.get_region(region))):
                raise EntranceShuffleError('Invalid starting area')

        # Check that a region where time passes is always reachable as both ages without having collected any items (except in closed forest)
        time_travel_search = Search.with_items([world.state for world in worlds], [ItemFactory('Time Travel', world=world) for world in worlds])

        for world in worlds:
            if not (any(region for region in time_travel_search.reachable_regions('child') if region.time_passes and region.world == world) and
                    any(region for region in time_travel_search.reachable_regions('adult') if region.time_passes and region.world == world)):
                raise EntranceShuffleError('Time passing is not guaranteed as both ages')

        # When starting as adult, child Link should be able to reach ToT without having collected any items
        # This is important to ensure that the player never loses access to the pedestal after going child
        if any(world.starting_age == 'adult' for world in worlds):
            for world in worlds:
                if world.starting_age == 'adult' and not time_travel_search.can_reach(world.get_region('Temple of Time'), age='child'):
                    raise EntranceShuffleError('Links House to Temple of Time path as child is not guaranteed')

    if entrance_placed == None or (entrance_placed != None and entrance_placed.type in ['Interior', 'SpecialInterior', 'Overworld']):
        # The Big Poe Shop should always be accessible as adult without the need to use any bottles
        # Since we can't guarantee that items in the pool won't be placed behind bottles, we guarantee the access without using any items
        # This is important to ensure that players can never lock their only bottles by filling them with Big Poes they can't sell
        no_items_time_travel_search = Search.with_items([State(world) for world in worlds], [ItemFactory('Time Travel', world=world) for world in worlds])

        for world in worlds:
            if not no_items_time_travel_search.can_reach(world.get_region('Market Guard House'), age='adult'):
                raise EntranceShuffleError('Big Poe Shop access is not guaranteed as adult')

            if world.shuffle_cows:
                impas_front_entrance = get_entrance_replacing(world.get_region('Kak Impas House'), 'Kakariko Village -> Kak Impas House')
                impas_back_entrance = get_entrance_replacing(world.get_region('Kak Impas House Back'), 'Kak Impas Ledge -> Kak Impas House Back')
                check_same_hint_region(impas_front_entrance, impas_back_entrance)

    return


# Shorthand function to check and validate that two entrances are in the same hint region
def check_same_hint_region(first, second):
    if  first.parent_region.hint is not None and second.parent_region.hint is not None and \
        first.parent_region.hint != second.parent_region.hint:
        raise EntranceShuffleError('Entrances are not in the same hint region')


# Shorthand function to find an entrance with the requested name leading to a specific region
def get_entrance_replacing(region, entrance_name):
    try:
        return next(filter(lambda entrance: entrance.replaces and entrance.replaces.name == entrance_name, region.entrances))
    except StopIteration:
        return region.world.get_entrance(entrance_name)


# Change connections between an entrance and a target assumed entrance, in order to test the connections afterwards if necessary
def change_connections(entrance, target_entrance):
    entrance.connect(target_entrance.disconnect())
    entrance.replaces = target_entrance.replaces
    if entrance.reverse:
        target_entrance.replaces.reverse.connect(entrance.reverse.assumed.disconnect())
        target_entrance.replaces.reverse.replaces = entrance.reverse.assumed.replaces


# Restore connections between an entrance and a target assumed entrance
def restore_connections(entrance, target_entrance):
    target_entrance.connect(entrance.disconnect())
    entrance.replaces = None
    if entrance.reverse:
        entrance.reverse.assumed.connect(target_entrance.replaces.reverse.disconnect())
        target_entrance.replaces.reverse.replaces = None


# Confirm the replacement of a target entrance by a new entrance, logging the new connections and completely deleting the target entrances
def confirm_replacement(entrance, target_entrance):
    delete_target_entrance(target_entrance)
    logging.getLogger('').debug('Connected %s To %s [World %d]', entrance, entrance.connected_region, entrance.world.id)
    if entrance.reverse:
        replaced_reverse = target_entrance.replaces.reverse
        delete_target_entrance(entrance.reverse.assumed)
        logging.getLogger('').debug('Connected %s To %s [World %d]', replaced_reverse, replaced_reverse.connected_region, replaced_reverse.world.id)


# Delete an assumed target entrance, by disconnecting it if needed and removing it from its parent region
def delete_target_entrance(target_entrance):
    if target_entrance.connected_region != None:
        target_entrance.disconnect()
    if target_entrance.parent_region != None:
        target_entrance.parent_region.exits.remove(target_entrance)
        target_entrance.parent_region = None
