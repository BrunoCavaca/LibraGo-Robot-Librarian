import random
import os
from gtts import gTTS
from pydub import AudioSegment

# Expanded list of example book titles (including common and obscure titles)
book_titles = [
    "The Great Gatsby",
    "To Kill a Mockingbird",
    "Moby-Dick",
    "War and Peace",
    "Crime and Punishment",
    "One Hundred Years of Solitude",
    "The Brothers Karamazov",
    "The Catcher in the Rye",
    "Invisible Man",
    "The Road",
    "Siddhartha",
    "The Night Circus",
    "The Secret History",
    "The Master and Margarita",
    "The Book Thief",
    "The Name of the Wind",
    "The Hitchhiker's Guide to the Galaxy",
    "House of Leaves",
    "Neverwhere",
    "Ficciones",
    "A Brief History of Time",
    "The Silence of the Lambs",
    "Brave New World",
    "1984",
    "The Picture of Dorian Gray",
    "The Alchemist",
    "The Girl with the Dragon Tattoo",
    "The Bell Jar",
    "American Gods",
    "The Shining",
    "The Handmaid's Tale",
    "The Road to Serfdom",
    "Catch-22",
    "The Grapes of Wrath",
    "The Goldfinch",
    "The Wind-Up Bird Chronicle",
    "The Subtle Art of Not Giving a F*ck",
    "The Hunger Games",
    "Life of Pi",
    "The Martian",
    "The Girl on the Train",
    "The Outsiders",
    "The Maze Runner",
    "The Chronicles of Narnia",
    "The Hobbit",
    "Ender's Game",
    "The Secret Garden",
    "Frankenstein"
]

# Expanded list of phrases (now with 50 phrases)
phrases = [
    "Can you get me {book}?",
    "I want to borrow {book}.",
    "Please bring me {book}.",
    "I need {book}, can you fetch it for me?",
    "Can you find {book} for me?",
    "Please retrieve {book} for me.",
    "I’m looking for {book}, do you have it?",
    "Could you get me {book} right away?",
    "I’d like to borrow {book}.",
    "Can you get me a copy of {book}?",
    "I want to read {book}, please bring it to me.",
    "Do you have {book}? I want to borrow it.",
    "Could you hand me {book}?",
    "Can I get {book} from you?",
    "Can you bring me {book}?",
    "Please get {book} for me.",
    "I need to borrow {book}.",
    "I’d love to have {book}, can you give it to me?",
    "I want to borrow {book}, is it available?",
    "Could you fetch me {book}?",
    "Please find me {book}.",
    "Can you retrieve {book} for me?",
    "I need {book}, could you bring it to me?",
    "Would you mind getting me {book}?",
    "Can you bring {book} to me?",
    "Could you find {book} for me?",
    "I need {book} now, could you bring it?",
    "Can you locate {book} for me?",
    "Please bring me {book}, it’s urgent.",
    "I’m looking for {book}, do you have a copy?",
    "Can I borrow {book} from you?",
    "Do you have {book} available?",
    "Please give me {book}.",
    "I’d like {book}, could you get it for me?",
    "Do you have {book} to lend?",
    "I need {book} right now.",
    "Can you get {book} for me, please?",
    "Could I borrow {book}?",
    "I need {book}, please find it for me.",
    "I want {book} now.",
    "Can you search for {book} and get it?",
    "Please provide {book}.",
    "Do you have {book}? I need it.",
    "Can I get {book} from the shelf?",
    "Can you go and get {book} for me?",
    "I want to borrow {book}, can you bring it?",
    "Could you get {book} and hand it to me?",
    "I want {book}, do you have it in stock?",
    "Can you locate {book} for me?"
]

# Expanded list of complex sentences (now with 50 complex sentences)
complex_sentences = [
    "I’m looking for {book}, could you locate a copy for me?",
    "I need {book}, can you provide it for me?",
    "I’m trying to get a hold of {book}, please find it for me.",
    "Please bring me {book} as soon as possible.",
    "I need to borrow {book}. Could you bring it to me?",
    "Can you help me find {book} in your collection?",
    "I need to get {book}, do you have it available?",
    "Can you fetch {book} from your shelves for me?",
    "Would it be possible for you to retrieve {book}?",
    "Could you find {book} in your catalog and bring it to me?",
    "I’m looking for {book}, could you check if you have it?",
    "Can you bring {book} from your library?",
    "Could you help me locate {book}?",
    "Can you provide me with a copy of {book}?",
    "I need {book}, please find it for me immediately.",
    "Please bring me {book}, I need it for my research.",
    "Do you have {book} available for borrowing?",
    "Can you get me {book} from your collection?",
    "Could you please find {book} for me and bring it here?",
    "I’m trying to locate {book}, could you help me with that?",
    "Could you fetch {book} for me from your database?",
    "I’m searching for {book}, could you check if it’s available?",
    "Can you bring me {book}? I need it for my study.",
    "I’d appreciate it if you could get me {book}.",
    "Can you assist me in getting {book} from your catalog?",
    "Please check your collection and bring me {book}.",
    "I need {book} for my project, can you fetch it?",
    "Can you locate {book} for me in your archives?",
    "Please bring {book}, I require it for my work.",
    "Could you retrieve {book} for me, it’s essential for my study?",
    "Can you locate {book} in your system and bring it to me?",
    "I’m requesting {book}, can you help with that?",
    "Could you bring me {book} right away? It’s urgent.",
    "Can you check if you have {book} and provide it?",
    "Please get me {book}, it’s needed immediately.",
    "Can you help me borrow {book} from your collection?",
    "Could you locate {book} for me and provide it?",
    "I need {book} right away, could you retrieve it?",
    "Please bring {book} to me, I need it urgently.",
    "Can you fetch {book} for me from the shelf?",
    "Could you go ahead and find {book} for me?",
    "Can you bring {book} to me as soon as possible?",
    "Please find {book} and get it for me.",
    "Could you get {book} from the library?",
    "I’d like to borrow {book}, could you provide it?",
    "Could you check if {book} is available for me?",
    "Please get {book}, I require it for my task."
]

simple_file = open("test/audio_tests/audio_files/simple_phrases/phrases_registered.txt","a")
complex_file = open("test/audio_tests/audio_files/complex_phrases/phrases_registered.txt","a")
for i in range(50):
    idx_book = random.randint(0,47)
    idx_phrase_simple = random.randint(0,48)
    txt_sim = phrases[idx_phrase_simple].format(book=book_titles[idx_book])
    language = 'en'
    formatString = txt_sim + "\n"
    gen_tts = gTTS(text=txt_sim,lang=language,slow=False)
    gen_tts.save(f"test/audio_tests/audio_files/simple_phrases/{i}.mp3")
    simple_file.write(formatString)
simple_file.close()

for i in range(50):
    idx_book = random.randint(0,46)
    idx_phrase_complex = random.randint(0,46)
    txt_sim = phrases[idx_phrase_complex].format(book=book_titles[idx_book])
    language = 'en'
    formatString = txt_sim + "\n"
    gen_tts = gTTS(text=txt_sim,lang=language,slow=False)
    gen_tts.save(f"test/audio_tests/audio_files/complex_phrases/{i}.mp3")
    complex_file.write(formatString)
complex_file.close()

# Save the MP3 file first (as you already do)
# Convert MP3 to WAV using pydub
for i in range(50):
    audio = AudioSegment.from_mp3(f"test/audio_tests/audio_files/simple_phrases/{i}.mp3")
    audio.export(f"test/audio_tests/audio_files/simple_phrases/{i}.wav", format="wav")
    os.remove(f"test/audio_tests/audio_files/simple_phrases/{i}.mp3")

for i in range(50):
    audio = AudioSegment.from_mp3(f"test/audio_tests/audio_files/complex_phrases/{i}.mp3")
    audio.export(f"test/audio_tests/audio_files/complex_phrases/{i}.wav", format="wav")
    os.remove(f"test/audio_tests/audio_files/complex_phrases/{i}.mp3")